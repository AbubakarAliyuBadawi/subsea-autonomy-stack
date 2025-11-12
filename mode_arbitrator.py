import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from datetime import datetime
import json

# Import our modules
from authority_rules import (
    PhaseRiskClassifier,
    AuthorityRuleEngine,
    ControlMode,
    ModeRecommendation,
    ActionType,
    AuthorityDecision
)


class ModeArbitratorNode(Node):
    """
    ROS2 node that arbitrates control mode based on:
    - BN recommendations
    - Mission phase
    - Phase-dependent authority rules
    - Operator input
    """
    
    def __init__(self):
        super().__init__('mode_arbitrator')
        
        # Authority rule engine
        self.rule_engine = AuthorityRuleEngine()
        
        # Current state
        self.current_mode = ControlMode.HUMAN  # Start in human control
        self.current_phase = "Transit"
        self.current_task_criticality = "Routine"
        self.last_mode_change_time = self.get_clock().now()
        
        # BN recommendation state
        self.latest_bn_recommendation = None
        self.human_reliability = 0.85
        self.autonomous_reliability = 0.80
        self.docking_reliability = None
        self.bn_confidence = 0.70
        
        # Pending decision tracking
        self.pending_decision = None
        self.decision_request_time = None
        
        # Logging
        self.mode_change_log = []
        
        # =====================
        # ROS2 Subscribers
        # =====================
        
        # BN outputs
        self.create_subscription(
            String,
            '/mission/bn_recommendation',
            self.bn_recommendation_callback,
            10
        )
        
        self.create_subscription(
            Float32,
            '/mission/human_reliability',
            self.human_reliability_callback,
            10
        )
        
        self.create_subscription(
            Float32,
            '/mission/autonomous_reliability',
            self.autonomous_reliability_callback,
            10
        )
        
        self.create_subscription(
            Float32,
            '/mission/docking_reliability',
            self.docking_reliability_callback,
            10
        )
        
        self.create_subscription(
            Float32,
            '/mission/bn_confidence',
            self.bn_confidence_callback,
            10
        )
        
        # Mission context
        self.create_subscription(
            String,
            '/mission/phase',
            self.mission_phase_callback,
            10
        )
        
        self.create_subscription(
            String,
            '/mission/task_criticality',
            self.task_criticality_callback,
            10
        )
        
        # Operator input
        self.create_subscription(
            String,
            '/operator/mode_override',
            self.operator_override_callback,
            10
        )
        
        self.create_subscription(
            Bool,
            '/operator/decision_response',
            self.operator_decision_response_callback,
            10
        )
        
        # =====================
        # ROS2 Publishers
        # =====================
        
        # Final control mode
        self.mode_publisher = self.create_publisher(
            String,
            '/mission/control_mode',
            10
        )
        
        # Operator interface messages
        self.ui_message_publisher = self.create_publisher(
            String,
            '/operator/ui_message',
            10
        )
        
        self.ui_request_publisher = self.create_publisher(
            String,
            '/operator/ui_request',
            10
        )
        
        # Status and logging
        self.status_publisher = self.create_publisher(
            String,
            '/mission/arbitrator_status',
            10
        )
        
        # =====================
        # Timers
        # =====================
        
        # Main arbitration loop (2 Hz)
        self.create_timer(0.5, self.arbitration_loop)
        
        # Check pending decisions (1 Hz)
        self.create_timer(1.0, self.check_pending_decisions)
        
        # Status reporting (0.2 Hz)
        self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('MODE ARBITRATOR NODE INITIALIZED')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Initial mode: {self.current_mode.name}')
        self.get_logger().info('Ready to receive BN recommendations and operator input')
        self.get_logger().info('=' * 70)
    
    # =====================
    # ROS2 Callbacks
    # =====================
    
    def bn_recommendation_callback(self, msg: String):
        """Receive BN mode recommendation"""
        mode_str = msg.data.lower()
        
        mode_map = {
            'autonomous': ControlMode.AUTONOMOUS,
            'human': ControlMode.HUMAN,
            'shared': ControlMode.SHARED
        }
        
        self.latest_bn_recommendation = mode_map.get(mode_str, ControlMode.SHARED)
    
    def human_reliability_callback(self, msg: Float32):
        """Receive human operator reliability"""
        self.human_reliability = msg.data
    
    def autonomous_reliability_callback(self, msg: Float32):
        """Receive autonomous system reliability"""
        self.autonomous_reliability = msg.data
    
    def docking_reliability_callback(self, msg: Float32):
        """Receive docking reliability"""
        self.docking_reliability = msg.data
    
    def bn_confidence_callback(self, msg: Float32):
        """Receive BN confidence in recommendation"""
        self.bn_confidence = msg.data
    
    def mission_phase_callback(self, msg: String):
        """Receive mission phase updates"""
        self.current_phase = msg.data
        self.get_logger().info(f'Mission phase changed to: {self.current_phase}')
    
    def task_criticality_callback(self, msg: String):
        """Receive task criticality updates"""
        self.current_task_criticality = msg.data
    
    def operator_override_callback(self, msg: String):
        """Handle operator manual override request"""
        requested_mode_str = msg.data.lower()
        
        mode_map = {
            'autonomous': ControlMode.AUTONOMOUS,
            'human': ControlMode.HUMAN,
            'shared': ControlMode.SHARED
        }
        
        requested_mode = mode_map.get(requested_mode_str)
        
        if requested_mode:
            self.get_logger().info(f'Operator override request: {requested_mode.name}')
            self._process_operator_override(requested_mode)
    
    def operator_decision_response_callback(self, msg: Bool):
        """Handle operator response to pending decision request"""
        if self.pending_decision is None:
            return
        
        accepted = msg.data
        
        self.get_logger().info(
            f'Operator {"ACCEPTED" if accepted else "DECLINED"} mode change to {self.pending_decision.target_mode.name}'
        )
        
        if accepted:
            self._execute_mode_change(
                self.pending_decision.target_mode,
                f"Operator accepted: {self.pending_decision.message}"
            )
        else:
            self._send_ui_message(
                f"Mode change declined. Maintaining {self.current_mode.name} mode.",
                "info"
            )
            
            # Log declined decision
            self._log_decision("operator_declined", self.pending_decision)
        
        # Clear pending decision
        self.pending_decision = None
        self.decision_request_time = None
    
    # =====================
    # Main Arbitration Logic
    # =====================
    
    def arbitration_loop(self):
        """Main arbitration logic loop"""
        
        # Skip if no BN recommendation yet
        if self.latest_bn_recommendation is None:
            return
        
        # Create BN recommendation object
        bn_rec = ModeRecommendation(
            recommended_mode=self.latest_bn_recommendation,
            confidence=self.bn_confidence,
            human_reliability=self.human_reliability,
            autonomous_reliability=self.autonomous_reliability,
            docking_reliability=self.docking_reliability
        )
        
        # Calculate time since last mode change
        current_time = self.get_clock().now()
        time_since_change = (current_time - self.last_mode_change_time).nanoseconds / 1e9
        
        # Evaluate mode change with rule engine
        decision = self.rule_engine.evaluate_mode_change(
            current_mode=self.current_mode,
            bn_recommendation=bn_rec,
            mission_phase=self.current_phase,
            task_criticality=self.current_task_criticality,
            time_since_last_change=time_since_change,
            operator_override=None  # Handled separately
        )
        
        # Execute decision
        self._execute_decision(decision)
    
    def _execute_decision(self, decision: AuthorityDecision):
        """Execute authority decision"""
        
        if decision.action_type == ActionType.NONE:
            # No action needed
            return
        
        elif decision.action_type == ActionType.AUTO_SWITCH:
            # Automatic mode change
            self._execute_mode_change(decision.target_mode, decision.message)
            self._send_ui_message(decision.explanation, decision.urgency)
            self._log_decision("auto_switch", decision)
        
        elif decision.action_type == ActionType.ASK:
            # Ask operator for confirmation
            self._request_operator_decision(decision)
            self._log_decision("asked_operator", decision)
        
        elif decision.action_type == ActionType.SUGGEST:
            # Suggest mode change (less urgent)
            self._send_ui_request(decision)
            self._log_decision("suggested", decision)
        
        elif decision.action_type == ActionType.NOTIFY:
            # Information only, no mode change
            self._send_ui_message(decision.explanation, decision.urgency)
            self._log_decision("notified", decision)
        
        elif decision.action_type == ActionType.BLOCK:
            # Block requested mode change
            self._send_ui_message(decision.explanation, "high")
            self._log_decision("blocked", decision)
    
    def _process_operator_override(self, requested_mode: ControlMode):
        """Process operator manual override request"""
        
        # Create BN recommendation for context
        bn_rec = ModeRecommendation(
            recommended_mode=self.latest_bn_recommendation if self.latest_bn_recommendation else requested_mode,
            confidence=self.bn_confidence,
            human_reliability=self.human_reliability,
            autonomous_reliability=self.autonomous_reliability,
            docking_reliability=self.docking_reliability
        )
        
        # Get risk level
        risk_level = PhaseRiskClassifier.get_risk_level(
            self.current_phase,
            self.current_task_criticality
        )
        
        # Handle override through rule engine
        decision = self.rule_engine._handle_operator_override(
            requested_mode,
            self.current_mode,
            risk_level,
            self.current_phase
        )
        
        self._execute_decision(decision)
    
    def _execute_mode_change(self, new_mode: ControlMode, reason: str):
        """Execute actual mode change"""
        
        old_mode = self.current_mode
        self.current_mode = new_mode
        self.last_mode_change_time = self.get_clock().now()
        
        # Publish new mode
        mode_msg = String()
        mode_msg.data = new_mode.name
        self.mode_publisher.publish(mode_msg)
        
        # Log
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'MODE CHANGE: {old_mode.name} → {new_mode.name}')
        self.get_logger().info(f'Reason: {reason}')
        self.get_logger().info(f'Phase: {self.current_phase}')
        self.get_logger().info('=' * 70)
        
        # Log to history
        self.mode_change_log.append({
            'timestamp': datetime.now().isoformat(),
            'old_mode': old_mode.name,
            'new_mode': new_mode.name,
            'phase': self.current_phase,
            'reason': reason,
            'human_reliability': self.human_reliability,
            'autonomous_reliability': self.autonomous_reliability
        })
    
    def _request_operator_decision(self, decision: AuthorityDecision):
        """Request operator decision on mode change"""
        
        # Store pending decision
        self.pending_decision = decision
        self.decision_request_time = self.get_clock().now()
        
        # Create request message
        request_data = {
            'action': 'mode_change_request',
            'current_mode': self.current_mode.name,
            'requested_mode': decision.target_mode.name,
            'message': decision.message,
            'explanation': decision.explanation,
            'urgency': decision.urgency,
            'allow_decline': decision.allow_decline,
            'timeout': decision.timeout_seconds,
            'phase': self.current_phase,
            'human_reliability': f"{self.human_reliability:.0%}",
            'autonomous_reliability': f"{self.autonomous_reliability:.0%}",
            'confidence': f"{self.bn_confidence:.0%}"
        }
        
        # Publish request
        msg = String()
        msg.data = json.dumps(request_data)
        self.ui_request_publisher.publish(msg)
        
        self.get_logger().info(f'Requested operator decision: {decision.message}')
    
    def _send_ui_request(self, decision: AuthorityDecision):
        """Send suggestion to operator (less intrusive than request)"""
        
        suggestion_data = {
            'action': 'mode_suggestion',
            'current_mode': self.current_mode.name,
            'suggested_mode': decision.target_mode.name,
            'message': decision.message,
            'explanation': decision.explanation,
            'urgency': decision.urgency
        }
        
        msg = String()
        msg.data = json.dumps(suggestion_data)
        self.ui_request_publisher.publish(msg)
    
    def _send_ui_message(self, message: str, urgency: str):
        """Send informational message to operator"""
        
        msg_data = {
            'action': 'info_message',
            'message': message,
            'urgency': urgency,
            'timestamp': datetime.now().isoformat()
        }
        
        msg = String()
        msg.data = json.dumps(msg_data)
        self.ui_message_publisher.publish(msg)
    
    def check_pending_decisions(self):
        """Check if pending decisions have timed out"""
        
        if self.pending_decision is None:
            return
        
        if self.decision_request_time is None:
            return
        
        timeout = self.pending_decision.timeout_seconds
        if timeout is None:
            return
        
        # Check if timeout exceeded
        current_time = self.get_clock().now()
        elapsed = (current_time - self.decision_request_time).nanoseconds / 1e9
        
        if elapsed > timeout:
            self.get_logger().warn(
                f'Operator decision timeout ({timeout}s) - Maintaining current mode'
            )
            
            self._send_ui_message(
                f"Decision timeout. Maintaining {self.current_mode.name} mode.",
                "medium"
            )
            
            # Log timeout
            self._log_decision("timeout", self.pending_decision)
            
            # Clear pending decision
            self.pending_decision = None
            self.decision_request_time = None
    
    def publish_status(self):
        """Publish current arbitrator status"""
        
        status_data = {
            'current_mode': self.current_mode.name,
            'mission_phase': self.current_phase,
            'task_criticality': self.current_task_criticality,
            'human_reliability': self.human_reliability,
            'autonomous_reliability': self.autonomous_reliability,
            'bn_recommendation': self.latest_bn_recommendation.name if self.latest_bn_recommendation else "None",
            'bn_confidence': self.bn_confidence,
            'pending_decision': self.pending_decision is not None,
            'mode_changes_count': len(self.mode_change_log)
        }
        
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)
    
    def _log_decision(self, decision_type: str, decision: AuthorityDecision):
        """Log decision for analysis"""
        
        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'decision_type': decision_type,
            'action_type': decision.action_type.value,
            'current_mode': self.current_mode.name,
            'target_mode': decision.target_mode.name,
            'phase': self.current_phase,
            'task_criticality': self.current_task_criticality,
            'human_reliability': self.human_reliability,
            'autonomous_reliability': self.autonomous_reliability,
            'bn_confidence': self.bn_confidence,
            'urgency': decision.urgency,
            'reasons': decision.reasons
        }
        
        # In full implementation, write to file or database
        # For now, just store in memory
        if not hasattr(self, 'decision_log'):
            self.decision_log = []
        
        self.decision_log.append(log_entry)
        
        # Optionally save to file periodically
        if len(self.decision_log) % 50 == 0:
            self._save_logs()
    
    def _save_logs(self):
        """Save logs to file"""
        try:
            with open('mode_arbitration_log.json', 'w') as f:
                json.dump({
                    'mode_changes': self.mode_change_log,
                    'decisions': self.decision_log
                }, f, indent=2)
            
            self.get_logger().info(f'Saved {len(self.decision_log)} decisions to log file')
        
        except Exception as e:
            self.get_logger().error(f'Failed to save logs: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = ModeArbitratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Mode Arbitrator...')
        node._save_logs()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()