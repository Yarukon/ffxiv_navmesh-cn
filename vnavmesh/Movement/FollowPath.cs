using System;
using System.Collections.Generic;
using System.Numerics;
using Dalamud.Game.ClientState.Conditions;
using Dalamud.Plugin;
using FFXIVClientStructs.FFXIV.Client.Game;
using Navmesh.Utilities;

namespace Navmesh.Movement;

public class FollowPath : IDisposable
{
    // 卡住恢复相关常量
    private const float GROUND_STUCK_THRESHOLD         = 1f; // 地面移动卡住检测阈值
    private const float FLYING_STUCK_THRESHOLD         = 3f; // 飞行移动卡住检测阈值
    private const float SIGNIFICANT_MOVEMENT_THRESHOLD = 3f; // 显著移动阈值
    private const int   STUCK_COUNTER_THRESHOLD        = 3;  // 连续几次检测卡住才判定为真卡住

    public bool          MovementAllowed = true;
    public bool          IgnoreDeltaY;
    public float         Tolerance = 0.25f;
    public List<Vector3> Waypoints = [];

    private readonly IDalamudPluginInterface _dalamud;
    private readonly NavmeshManager          _manager;
    private readonly OverrideCamera          _camera   = new();
    private readonly OverrideMovement        _movement = new();
    private          DateTime                _nextJump;

    private Vector3? posPreviousFrame;

    // 卡住检测相关
    private          Vector3  _lastSignificantPosition;
    private          DateTime _lastMovementTime;
    private          DateTime _lastStuckCheckTime;
    private          int      _stuckCounter;
    private          bool     _isStuck;
    private readonly TimeSpan _stuckCheckInterval = TimeSpan.FromSeconds(1f); // 卡住检测间隔
    private readonly TimeSpan _stuckTimeout       = TimeSpan.FromSeconds(2f); // 判定为卡住的时间阈值
    private          int      _recoveryAttempts;
    private          DateTime _nextRecoveryTime;
    private          Vector3? _recoveryDirection;
    private          DateTime _stuckStartTime;                                       // 记录开始卡住的时间
    private          bool     _inRecoveryProcess;                                    // 标记是否正在恢复过程中
    private          DateTime _lastSignificantMovementTime;                          // 上次有显著移动的时间
    private          bool     _pathRecalculationRequested;                           // 是否已请求重新计算路径
    private          Vector3? _originalTargetPosition;                               // 原始目标位置
    private          Vector3? _alternativeTargetPosition;                            // 备选目标位置
    private          DateTime _lastPathRecalculationTime;                            // 上次重新计算路径的时间
    private readonly TimeSpan _pathRecalculationCooldown = TimeSpan.FromSeconds(5f); // 路径重新计算冷却时间
    private          bool     _isFlying;                                             // 当前是否处于飞行状态

    public event Action<Vector3, Vector3, bool>? RequestPathRecalculation;

    // entries in dalamud shared data cache must be reference types, so we use an array
    private readonly bool[] _sharedPathIsRunning;

    private const string _sharedPathTag = "vnav.PathIsRunning";

    public FollowPath(IDalamudPluginInterface dalamud, NavmeshManager manager)
    {
        _dalamud                  =  dalamud;
        _sharedPathIsRunning      =  _dalamud.GetOrCreateData<bool[]>(_sharedPathTag, () => [false]);
        _manager                  =  manager;
        _manager.OnNavmeshChanged += OnNavmeshChanged;
        OnNavmeshChanged(_manager.Navmesh, _manager.Query);
        _lastMovementTime            = DateTime.Now;
        _lastStuckCheckTime          = DateTime.Now;
        _nextRecoveryTime            = DateTime.Now;
        _lastSignificantMovementTime = DateTime.Now;
        _lastPathRecalculationTime   = DateTime.Now;
    }

    public void Dispose()
    {
        UpdateSharedState(false);
        _dalamud.RelinquishData(_sharedPathTag);
        _manager.OnNavmeshChanged -= OnNavmeshChanged;
        _camera.Dispose();
        _movement.Dispose();
    }

    private void UpdateSharedState(bool isRunning) => _sharedPathIsRunning[0] = isRunning;

    public void Update()
    {
        var player = Service.ClientState.LocalPlayer;
        if (player == null) return;

        // 更新飞行状态
        _isFlying = Service.Condition[ConditionFlag.InFlight] || Service.Condition[ConditionFlag.Diving] || !IgnoreDeltaY;

        while (Waypoints.Count > 0)
        {
            var a = Waypoints[0];
            var b = player.Position;
            var c = posPreviousFrame ?? b;

            if (IgnoreDeltaY)
            {
                a.Y = 0;
                b.Y = 0;
                c.Y = 0;
            }

            if (DistanceToLineSegment(a, b, c) > Tolerance)
                break;

            Waypoints.RemoveAt(0);
        }

        // 检测卡住情况
        var now        = DateTime.Now;
        var currentPos = player.Position;

        if (posPreviousFrame.HasValue && Waypoints.Count > 0)
        {
            CheckForStuck(currentPos, now);

            // 如果检测到卡住，尝试恢复
            if (_isStuck)
            {
                TryRecoverFromStuck(currentPos, now);
            }
            else if (!_inRecoveryProcess)
            {
                // 只有在非恢复过程中才重置恢复相关变量
                _recoveryAttempts           = 0;
                _recoveryDirection          = null;
                _pathRecalculationRequested = false;
                _originalTargetPosition     = null;
                _alternativeTargetPosition  = null;
            }
        }

        posPreviousFrame = currentPos;

        if (Waypoints.Count == 0)
        {
            _movement.Enabled         = _camera.Enabled = false;
            _camera.SpeedH            = _camera.SpeedV  = default;
            _movement.DesiredPosition = player.Position;
            UpdateSharedState(false);

            // 重置卡住状态
            ResetStuckState();
        }
        else
        {
            if (Service.Config.CancelMoveOnUserInput && _movement.UserInput)
            {
                Stop();
                return;
            }

            OverrideAFK.ResetTimers();
            _movement.Enabled = MovementAllowed;

            // 如果正在执行恢复动作，使用恢复方向
            if (_isStuck && _recoveryDirection.HasValue && now < _nextRecoveryTime)
                _movement.DesiredPosition = player.Position + _recoveryDirection.Value;
            else
                _movement.DesiredPosition = Waypoints[0];

            if (_movement.DesiredPosition.Y > player.Position.Y && !Service.Condition[ConditionFlag.InFlight] && !Service.Condition[ConditionFlag.Diving] &&
                !IgnoreDeltaY) //Only do this bit if on a flying path
            {
                // walk->fly transition (TODO: reconsider?)
                if (Service.Condition[ConditionFlag.Mounted])
                    ExecuteJump(); // Spam jump to take off
                else
                {
                    _movement.Enabled = false; // Don't move, since it'll just run on the spot
                    return;
                }
            }

            _camera.Enabled         = Service.Config.AlignCameraToMovement;
            _camera.SpeedH          = _camera.SpeedV = 360.Degrees();
            _camera.DesiredAzimuth  = Angle.FromDirectionXZ(_movement.DesiredPosition - player.Position) + 180.Degrees();
            _camera.DesiredAltitude = -30.Degrees();
        }
    }

    private void CheckForStuck(Vector3 currentPos, DateTime now)
    {
        // 如果正在计算新路径，暂时不进行卡住检测，防止重复判断
        if (Plugin.Instance().AsyncMove.TaskInBusy || 
            (_pathRecalculationRequested && now - _lastPathRecalculationTime < TimeSpan.FromSeconds(3.0)))
        {
            // 如果正在计算路径，重置卡住检测计时器，防止误判
            _lastStuckCheckTime = now;
            _lastMovementTime = now;
            return;
        }

        // 每隔一段时间检查一次是否卡住
        if (now - _lastStuckCheckTime < _stuckCheckInterval)
            return;

        _lastStuckCheckTime = now;

        // 根据当前移动模式选择适当的阈值
        var stuckThreshold = _isFlying ? FLYING_STUCK_THRESHOLD : GROUND_STUCK_THRESHOLD;

        // 计算距离上次显著位置的移动距离
        var movementDistance = IgnoreDeltaY
                                   ? Vector2.Distance(new Vector2(currentPos.X, currentPos.Z), new Vector2(_lastSignificantPosition.X, _lastSignificantPosition.Z))
                                   : Vector3.Distance(currentPos, _lastSignificantPosition);

        // 检测是否有显著移动（超过显著移动阈值）
        var hasSignificantMovement = movementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD;
        if (hasSignificantMovement)
        {
            _lastSignificantMovementTime = now;
            
            // 只有非恢复过程中或者有显著移动时才重置卡住状态
            if (!_inRecoveryProcess)
            {
                _lastSignificantPosition = currentPos;
                _lastMovementTime        = now;
                _stuckCounter            = 0;
                _isStuck                 = false;
            }
        }

        // 如果移动距离小于阈值，可能卡住了
        if (movementDistance < stuckThreshold)
        {
            _stuckCounter++;

            // 如果连续多次检测都没有移动，且超过了超时时间，则判定为卡住
            if (_stuckCounter >= STUCK_COUNTER_THRESHOLD && now - _lastMovementTime > _stuckTimeout)
            {
                if (!_isStuck)
                {
                    // 再次检查是否在计算路径，防止误判
                    if (Plugin.Instance().AsyncMove.TaskInBusy)
                    {
                        Service.Log.Debug("检测到可能卡住，但正在计算路径，暂不判定为卡住");
                        _stuckCounter--;
                        return;
                    }
                    
                    Service.Log.Debug($"检测到导航卡住: 位置 {currentPos}, 未移动时间: {(now - _lastMovementTime).TotalSeconds:F1}秒, 移动模式: {(_isFlying ? "飞行" : "地面")}");
                    _isStuck           = true;
                    _stuckStartTime    = now;
                    _inRecoveryProcess = true;

                    // 记录原始目标位置
                    if (Waypoints.Count > 0 && _originalTargetPosition == null) _originalTargetPosition = Waypoints[^1]; // 保存最终目标点
                }
            }
        }
        else if (!_inRecoveryProcess)
        {
            // 仅在非恢复过程中更新位置和时间
            _lastSignificantPosition = currentPos;
            _lastMovementTime        = now;
            
            // 只有移动大于阈值且不在恢复过程中，才减少计数器
            if (_stuckCounter > 0)
                _stuckCounter--;
        }
        
        // 检查是否长时间卡住（超过8秒），无论是否有微小移动
        if (_isStuck && now - _stuckStartTime > TimeSpan.FromSeconds(8) && now - _lastSignificantMovementTime > TimeSpan.FromSeconds(4))
        {
            // 确保不在路径计算中再触发重新计算
            if (!Plugin.Instance().AsyncMove.TaskInBusy)
            {
                // 长时间卡住，直接重新计算路径
                TryRecalculatePath(currentPos, now, true);
            }
        }
    }

    private void TryRecalculatePath(Vector3 currentPos, DateTime now, bool forceFindAlternative = false)
    {
        if (_pathRecalculationRequested && !forceFindAlternative) return;
        
        if (now - _lastPathRecalculationTime < _pathRecalculationCooldown && !forceFindAlternative)
            return;

        // 如果已经在计算路径，不要重复请求
        if (Plugin.Instance().AsyncMove.TaskInBusy)
        {
            Service.Log.Debug("正在计算路径中，忽略重新计算请求");
            return;
        }

        _lastPathRecalculationTime = now;

        if (_originalTargetPosition.HasValue)
        {
            var targetPos = _originalTargetPosition.Value;

            if (forceFindAlternative || _alternativeTargetPosition == null)
            {
                var random  = new Random();
                var offsetX = (float)((random.NextDouble() * 5.0) - 2.5);
                var offsetZ = (float)((random.NextDouble() * 5.0) - 2.5);
                var offsetY = _isFlying ? (float)((random.NextDouble() * 4.0) - 2.0) : 0; // 飞行时才考虑Y轴偏移

                _alternativeTargetPosition = new Vector3(
                    targetPos.X + offsetX,
                    targetPos.Y + offsetY, // 飞行模式下Y轴也添加随机偏移
                    targetPos.Z + offsetZ
                );
                
                Service.Log.Debug($"生成备选目标点: 原始{targetPos} -> 备选{_alternativeTargetPosition}, 移动模式: {(_isFlying ? "飞行" : "地面")}");
            }
            
            var destination = forceFindAlternative && _alternativeTargetPosition.HasValue ? _alternativeTargetPosition.Value : targetPos;
            
            // 触发路径重新计算事件
            Service.Log.Debug($"请求重新计算路径: 从 {currentPos} 到 {destination}, 移动模式: {(_isFlying ? "飞行" : "地面")}");
            RequestPathRecalculation?.Invoke(currentPos, destination, IgnoreDeltaY);
            _pathRecalculationRequested = true;
            
            // 设置较长的恢复时间，等待路径计算完成
            _nextRecoveryTime  = now.AddSeconds(2.5); // 增加等待时间，确保有足够时间计算路径
            _recoveryDirection = null;
            
            // 重置卡住检测的计时器，避免在计算路径期间触发卡住判定
            _lastMovementTime = now;
        }
    }

    private void TryRecoverFromStuck(Vector3 currentPos, DateTime now)
    {
        if (now < _nextRecoveryTime) return;

        _recoveryAttempts++;
        _inRecoveryProcess = true;

        // 根据当前是否飞行选择不同的恢复策略
        if (_isFlying)
            RecoverFromFlyingStuck(currentPos, now);
        else
            RecoverFromGroundStuck(currentPos, now);
    }

    // 飞行状态下的恢复策略
    private void RecoverFromFlyingStuck(Vector3 currentPos, DateTime now)
    {
        switch (_recoveryAttempts)
        {
            // 第一次尝试：垂直上升一段距离
            case 1:
                _recoveryDirection = new Vector3(0, 5.0f, 0);
                Service.Log.Debug("飞行模式：尝试向上升高");
                _nextRecoveryTime = now.AddSeconds(1f);
                break;
            // 第二次尝试：原地盘旋后上升
            case 2:
            {
                const float spiralAngle = 90.0f * (float)Math.PI / 180.0f;
                var         direction   = Waypoints.Count > 0 ? Vector3.Normalize(Waypoints[0] - currentPos) : Vector3.UnitZ;

                _recoveryDirection = new Vector3(
                                         (direction.X * (float)Math.Cos(spiralAngle)) - (direction.Z * (float)Math.Sin(spiralAngle)),
                                         3.0f, // 向上升高
                                         (direction.X * (float)Math.Sin(spiralAngle)) + (direction.Z * (float)Math.Cos(spiralAngle))) * 3.0f;

                Service.Log.Debug("飞行模式：尝试盘旋上升");
                _nextRecoveryTime = now.AddSeconds(1f);
            }
                break;
            // 第三次尝试：重新计算路径
            case 3:
                Service.Log.Debug("飞行模式：尝试重新计算路径");
                TryRecalculatePath(currentPos, now);
                break;
            // 第四次尝试：先向后退再向上升高，然后重新计算
            case 4:
            {
                var direction = Waypoints.Count > 0 ? Vector3.Normalize(currentPos - Waypoints[0]) : -Vector3.UnitZ;
                direction.Y        = 2.0f;
                _recoveryDirection = direction * 5.0f;

                Service.Log.Debug("飞行模式：尝试后退上升");
                _nextRecoveryTime = now.AddSeconds(2.0);

                // 在下一步后自动重新计算路径
                _pathRecalculationRequested = false;
            }
                break;
            // 第五次尝试：强制使用备选目标点重新计算高空路径
            case 5:
                Service.Log.Debug("飞行模式：使用备选高空目标点重新计算");
                TryRecalculatePath(currentPos, now, true);
                break;
            // 多次尝试失败，停止导航
            default:
                Service.Log.Error("多次飞行恢复尝试失败，停止导航");
                Stop();
                break;
        }
    }

    // 地面行走状态的恢复策略
    private void RecoverFromGroundStuck(Vector3 currentPos, DateTime now)
    {
        switch (_recoveryAttempts)
        {
            case 1:
                Service.Log.Debug("地面模式：尝试跳跃");
                ExecuteJump();
                
                _nextRecoveryTime        = now.AddSeconds(2.0);
                _lastSignificantPosition = currentPos;
                break;

            case 2:
                var jumpMovementDistance = Vector3.Distance(currentPos, _lastSignificantPosition);
                if (jumpMovementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD * 0.5f)
                {
                    Service.Log.Debug($"跳跃似乎有效，移动了 {jumpMovementDistance:F2} 距离，重置卡住状态");
                    
                    // 重置卡住状态，但保留当前位置作为新的参考点
                    _isStuck                 = false;
                    _stuckCounter            = 0;
                    _recoveryAttempts        = 0;
                    _inRecoveryProcess       = false;
                    _lastMovementTime        = now;
                    _lastSignificantPosition = currentPos;
                    return;
                }
                
                // 跳跃效果不明显，尝试左转移动
                const float leftAngle = 45.0f * (float)Math.PI / 180.0f;
                var direction = Waypoints.Count > 0 ? Vector3.Normalize(Waypoints[0] - currentPos) : Vector3.UnitZ;
                
                _recoveryDirection = new Vector3((direction.X * (float)Math.Cos(leftAngle)) - (direction.Z * (float)Math.Sin(leftAngle)), 0,
                                                 (direction.X * (float)Math.Sin(leftAngle)) + (direction.Z * (float)Math.Cos(leftAngle))) * 2.0f;
                
                Service.Log.Debug("地面模式：尝试左侧移动");
                _nextRecoveryTime = now.AddSeconds(1.5);
                _lastSignificantPosition = currentPos; // 更新参考位置
                break;

            case 3:
                // 检查左转移动是否有效
                var leftMovementDistance = Vector3.Distance(currentPos, _lastSignificantPosition);
                
                if (leftMovementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD * 0.5f)
                {
                    Service.Log.Debug($"左侧移动有效，移动了{leftMovementDistance:F2}距离，重置卡住状态");
                    ResetStuckState();
                    _lastMovementTime = now;
                    _lastSignificantPosition = currentPos;
                    return;
                }
                
                // 左转移动效果不明显，尝试右转移动
                const float rightAngle = -45.0f * (float)Math.PI / 180.0f;
                direction = Waypoints.Count > 0 ? Vector3.Normalize(Waypoints[0] - currentPos) : Vector3.UnitZ;
                
                _recoveryDirection = new Vector3(
                    (direction.X * (float)Math.Cos(rightAngle)) - (direction.Z * (float)Math.Sin(rightAngle)),
                    0,
                    (direction.X * (float)Math.Sin(rightAngle)) + (direction.Z * (float)Math.Cos(rightAngle))
                ) * 2.0f;
                
                Service.Log.Debug("地面模式：尝试右侧移动");
                _nextRecoveryTime = now.AddSeconds(1.5);
                _lastSignificantPosition = currentPos;
                break;

            case 4:
                // 检查右转移动是否有效
                var rightMovementDistance = Vector3.Distance(currentPos, _lastSignificantPosition);
                
                if (rightMovementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD * 0.5f)
                {
                    Service.Log.Debug($"右侧移动有效，移动了{rightMovementDistance:F2}距离，重置卡住状态");
                    ResetStuckState();
                    _lastMovementTime = now;
                    _lastSignificantPosition = currentPos;
                    return;
                }
                
                // 之前的微调移动都无效，尝试重新计算路径
                Service.Log.Debug("地面模式：方向微调未奏效，尝试重新计算路径");
                TryRecalculatePath(currentPos, now);
                break;

            case 5:
                // 检查新路径是否已有效
                if (!Plugin.Instance().AsyncMove.TaskInBusy && now - _lastPathRecalculationTime > TimeSpan.FromSeconds(3.0))
                {
                    // 新路径加载后但仍卡住，尝试后退
                    var backDirection = Waypoints.Count > 0 ? Vector3.Normalize(currentPos - Waypoints[0]) : -Vector3.UnitZ;
                    _recoveryDirection = backDirection * 3.0f;
                    
                    Service.Log.Debug("地面模式：尝试后退");
                    _nextRecoveryTime = now.AddSeconds(2.0);
                    _lastSignificantPosition = currentPos;
                }
                else
                {
                    // 路径计算中或刚计算完，多等一会
                    _nextRecoveryTime = now.AddSeconds(1.0);
                }
                break;

            case 6:
                // 检查后退是否有效
                var backMovementDistance = Vector3.Distance(currentPos, _lastSignificantPosition);
                
                if (backMovementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD * 0.5f)
                {
                    Service.Log.Debug($"后退移动有效，移动了{backMovementDistance:F2}距离，重置卡住状态");
                    ResetStuckState();
                    _lastMovementTime = now;
                    _lastSignificantPosition = currentPos;
                    return;
                }
                
                // 后退无效，尝试使用备选目标点重新计算路径
                Service.Log.Debug("地面模式：后退未奏效，使用备选目标点重新计算");
                TryRecalculatePath(currentPos, now, true);
                break;

            case 7:
                // 第七次尝试：跳跃+向前冲刺组合
                Service.Log.Debug("地面模式：尝试跳跃冲刺组合");
                ExecuteJump();
                
                if (Waypoints.Count > 0)
                {
                    var forwardDirection = Vector3.Normalize(Waypoints[0] - currentPos);
                    _recoveryDirection = forwardDirection * 4.0f; // 使用更大的冲量
                }
                
                _nextRecoveryTime = now.AddSeconds(1.5);
                _lastSignificantPosition = currentPos;
                break;

            case 8:
                // 检查跳跃冲刺是否有效
                var finalMovementDistance = Vector3.Distance(currentPos, _lastSignificantPosition);
                
                if (finalMovementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD * 0.5f)
                {
                    Service.Log.Debug($"跳跃冲刺有效，移动了{finalMovementDistance:F2}距离，重置卡住状态");
                    ResetStuckState();
                    _lastMovementTime = now;
                    _lastSignificantPosition = currentPos;
                    return;
                }
                
                // 所有策略都失败，停止导航
                Service.Log.Error("多次地面恢复尝试失败，停止导航");
                Stop();
                break;

            default:
                // 防止意外情况，停止导航
                Service.Log.Error($"未知恢复阶段({_recoveryAttempts})，停止导航");
                Stop();
                break;
        }
    }

    private void ResetStuckState()
    {
        _isStuck                    = false;
        _stuckCounter               = 0;
        _recoveryAttempts           = 0;
        _recoveryDirection          = null;
        _inRecoveryProcess          = false;
        _pathRecalculationRequested = false;
        _originalTargetPosition     = null;
        _alternativeTargetPosition  = null;
    }

    private static float DistanceToLineSegment(Vector3 v, Vector3 a, Vector3 b)
    {
        var ab = b - a;
        var av = v - a;

        if (ab.Length() == 0 || Vector3.Dot(av, ab) <= 0)
            return av.Length();

        var bv = v - b;
        if (Vector3.Dot(bv, ab) >= 0)
            return bv.Length();

        return Vector3.Cross(ab, av).Length() / ab.Length();
    }

    public void Stop()
    {
        UpdateSharedState(false);
        Waypoints.Clear();
        ResetStuckState();
    }

    private unsafe void ExecuteJump()
    {
        if (Service.Condition[ConditionFlag.Diving]) return;
        if (DateTime.Now >= _nextJump)
        {
            ActionManager.Instance()->UseAction(ActionType.GeneralAction, 2);
            _nextJump = DateTime.Now.AddMilliseconds(100);
        }
    }

    public void Move(List<Vector3> waypoints, bool ignoreDeltaY)
    {
        UpdateSharedState(true);
        Waypoints    = waypoints;
        IgnoreDeltaY = ignoreDeltaY;

        // 重置卡住检测状态
        ResetStuckState();

        var player = Service.ClientState.LocalPlayer;
        if (player != null)
        {
            _lastSignificantPosition     = player.Position;
            _lastMovementTime            = DateTime.Now;
            _lastSignificantMovementTime = DateTime.Now;
        }
    }

    private void OnNavmeshChanged(Navmesh? navmesh, NavmeshQuery? query)
    {
        UpdateSharedState(false);
        Waypoints.Clear();
    }
}
