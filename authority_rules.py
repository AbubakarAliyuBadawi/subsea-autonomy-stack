from enum import Enum
from dataclasses import dataclass
from typing import Dict, Tuple, Optional

class PhaseRiskLevel(Enum):
    """Risk categorization for mission phases"""
    SAFE = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4

class ControlMode(Enum):
    """Available control modes"""
    AUTONOMOUS = 0
    HUMAN = 1
    SHARED = 2

class ActionType(Enum):
    """Types of actions the system can take"""
    AUTO_SWITCH = "auto_switch"           # Automatic mode change (no confirmation)
    ASK = "ask"                           # Request with Yes/No
    SUGGEST = "suggest"                   # Suggestion (less urgent)
    NOTIFY = "notify"                     # Information only
    BLOCK = "block"                       # Prevent mode change
    NONE = "none"                         # No action needed

@dataclass
class ModeRecommendation:
    """System recommendation for control mode"""
    recommended_mode: ControlMode
    confidence: float
    human_reliability: float
    autonomous_reliability: float
    docking_reliability: Optional[float] = None

@dataclass
class AuthorityDecision:
    """Decision output from authority rule engine"""
    action_type: ActionType
    target_mode: ControlMode
    message: str
    explanation: str
    urgency: str  # "low", "medium", "high", "critical"
    allow_decline: bool
    timeout_seconds: Optional[int]
    reasons: Dict[str, any]  # Detailed reasons for logging/analysis


class PhaseRiskClassifier:
    """Classifies mission phases by risk level"""
    
    # Phase risk mapping
    PHASE_RISK_MAP = {
        "Docking": PhaseRiskLevel.CRITICAL,
        "DockingApproach": PhaseRiskLevel.CRITICAL,
        "Undocking": PhaseRiskLevel.HIGH,
        "Inspection": PhaseRiskLevel.MEDIUM,
        "Transit": PhaseRiskLevel.LOW,
        "Charging": PhaseRiskLevel.SAFE
    }
    
    # Task criticality modifiers
    CRITICALITY_MODIFIER = {
        "Routine": 0,
        "Important": 1,
        "Critical": 2
    }
    
    @classmethod
    def get_risk_level(cls, phase: str, task_criticality: str = "Routine") -> PhaseRiskLevel:
        """
        Get effective risk level considering both phase and task criticality
        
        Args:
            phase: Mission phase name
            task_criticality: Task criticality level
            
        Returns:
            Effective risk level
        """
        base_risk = cls.PHASE_RISK_MAP.get(phase, PhaseRiskLevel.MEDIUM)
        modifier = cls.CRITICALITY_MODIFIER.get(task_criticality, 0)
        
        # Increase risk level if task is critical
        effective_risk_value = min(base_risk.value + modifier, PhaseRiskLevel.CRITICAL.value)
        
        return PhaseRiskLevel(effective_risk_value)
    
    @classmethod
    def is_autonomous_allowed(cls, phase: str, task_criticality: str = "Routine") -> bool:
        """
        Check if autonomous mode is allowed in this phase
        
        Critical phases (Docking, DockingApproach) never allow autonomous mode
        """
        risk_level = cls.get_risk_level(phase, task_criticality)
        return risk_level != PhaseRiskLevel.CRITICAL


class AuthorityRuleEngine:
    """
    Implements phase-dependent authority rules for mode arbitration
    
    Priority hierarchy:
    1. Hard safety rules (e.g., no autonomous during docking)
    2. Operator override (always honored)
    3. Phase-dependent rules
    4. BN recommendation
    5. Default: maintain current mode
    """
    
    def __init__(self):
        self.mode_change_history = []
        self.last_mode_change_time = None
        self.minimum_mode_duration = 120.0  # seconds (2 minutes)
        
        # Reliability thresholds
        self.CRITICAL_LOW_THRESHOLD = 0.5
        self.LOW_THRESHOLD = 0.6
        self.HIGH_THRESHOLD = 0.8
    
    def evaluate_mode_change(
        self,
        current_mode: ControlMode,
        bn_recommendation: ModeRecommendation,
        mission_phase: str,
        task_criticality: str,
        time_since_last_change: float,
        operator_override: Optional[ControlMode] = None
    ) -> AuthorityDecision:
        """
        Evaluate whether mode change should occur and how
        
        Args:
            current_mode: Current control mode
            bn_recommendation: BN recommendation
            mission_phase: Current mission phase
            task_criticality: Task criticality level
            time_since_last_change: Time since last mode change (seconds)
            operator_override: Operator manual override (if any)
            
        Returns:
            AuthorityDecision with action to take
        """
        
        recommended_mode = bn_recommendation.recommended_mode
        risk_level = PhaseRiskClassifier.get_risk_level(mission_phase, task_criticality)
        
        # Priority 1: Operator Override
        if operator_override is not None:
            return self._handle_operator_override(
                operator_override, 
                current_mode,
                risk_level,
                mission_phase
            )
        
        # Priority 2: Hard Safety Rules
        safety_decision = self._apply_safety_rules(
            current_mode,
            recommended_mode,
            risk_level,
            mission_phase,
            bn_recommendation
        )
        if safety_decision:
            return safety_decision
        
        # Priority 3: Check minimum mode duration (prevent oscillation)
        if time_since_last_change < self.minimum_mode_duration:
            if recommended_mode != current_mode:
                return AuthorityDecision(
                    action_type=ActionType.NONE,
                    target_mode=current_mode,
                    message=f"Mode stable for {int(time_since_last_change)}s",
                    explanation="Minimum mode duration not reached to prevent rapid switching",
                    urgency="low",
                    allow_decline=False,
                    timeout_seconds=None,
                    reasons={"hysteresis": True, "time_remaining": self.minimum_mode_duration - time_since_last_change}
                )
        
        # Priority 4: Phase-Dependent Rules
        return self._apply_phase_rules(
            current_mode,
            recommended_mode,
            risk_level,
            mission_phase,
            bn_recommendation
        )
    
    def _handle_operator_override(
        self,
        requested_mode: ControlMode,
        current_mode: ControlMode,
        risk_level: PhaseRiskLevel,
        mission_phase: str
    ) -> AuthorityDecision:
        """Handle operator manual override requests"""
        
        # Check if requested mode is allowed in this phase
        if requested_mode == ControlMode.AUTONOMOUS:
            if not PhaseRiskClassifier.is_autonomous_allowed(mission_phase):
                return AuthorityDecision(
                    action_type=ActionType.BLOCK,
                    target_mode=current_mode,
                    message=f"❌ Autonomous mode not allowed during {mission_phase}",
                    explanation=f"Safety rules prohibit autonomous operation during {mission_phase} phase. "
                                f"This is a critical phase requiring human judgment. "
                                f"Shared control is available if you need assistance.",
                    urgency="high",
                    allow_decline=False,
                    timeout_seconds=None,
                    reasons={
                        "blocked": True,
                        "reason": "phase_safety_rule",
                        "phase": mission_phase,
                        "risk_level": risk_level.name,
                        "alternative": "Shared control available"
                    }
                )
        
        # Allow override with appropriate warning
        urgency_map = {
            PhaseRiskLevel.SAFE: "low",
            PhaseRiskLevel.LOW: "low",
            PhaseRiskLevel.MEDIUM: "medium",
            PhaseRiskLevel.HIGH: "high",
            PhaseRiskLevel.CRITICAL: "critical"
        }
        
        warning = ""
        if risk_level.value >= PhaseRiskLevel.HIGH.value:
            warning = f" ⚠️ Warning: {mission_phase} is a high-risk phase."
        
        return AuthorityDecision(
            action_type=ActionType.AUTO_SWITCH,
            target_mode=requested_mode,
            message=f"✓ Operator override accepted: {requested_mode.name}{warning}",
            explanation=f"You have manually selected {requested_mode.name} mode. "
                        f"The system will respect your decision.{warning}",
            urgency=urgency_map[risk_level],
            allow_decline=False,
            timeout_seconds=None,
            reasons={
                "operator_override": True,
                "risk_level": risk_level.name,
                "phase": mission_phase
            }
        )
    
    def _apply_safety_rules(
        self,
        current_mode: ControlMode,
        recommended_mode: ControlMode,
        risk_level: PhaseRiskLevel,
        mission_phase: str,
        bn_rec: ModeRecommendation
    ) -> Optional[AuthorityDecision]:
        """
        Apply hard safety rules
        Returns decision if rule triggers, None otherwise
        """
        
        # SAFETY RULE 1: Critically low human reliability during critical phase
        if (risk_level == PhaseRiskLevel.CRITICAL and 
            current_mode == ControlMode.HUMAN and
            bn_rec.human_reliability < self.CRITICAL_LOW_THRESHOLD):
            
            return AuthorityDecision(
                action_type=ActionType.NOTIFY,
                target_mode=current_mode,
                message=f"🚨 SAFETY ALERT: Critical fatigue during {mission_phase}",
                explanation=f"Your reliability is critically low ({bn_rec.human_reliability:.2f}) during {mission_phase} phase. "
                            f"Consider aborting operation and returning to safe zone. "
                            f"Autonomous mode is not allowed during this critical phase. "
                            f"Shared control can provide assistance.",
                urgency="critical",
                allow_decline=False,
                timeout_seconds=None,
                reasons={
                    "safety_alert": True,
                    "human_reliability": bn_rec.human_reliability,
                    "threshold": self.CRITICAL_LOW_THRESHOLD,
                    "phase": mission_phase,
                    "recommendation": "abort_and_rest"
                }
            )
        
        # SAFETY RULE 2: Autonomous wants to switch to human during degradation
        if (current_mode == ControlMode.AUTONOMOUS and
            recommended_mode == ControlMode.HUMAN and
            bn_rec.autonomous_reliability < self.LOW_THRESHOLD):
            
            return AuthorityDecision(
                action_type=ActionType.AUTO_SWITCH,
                target_mode=ControlMode.HUMAN,
                message="⚠️ Autonomous performance degraded - Switching to Human",
                explanation=f"Autonomous control reliability has dropped to {bn_rec.autonomous_reliability:.2f}. "
                            f"Reasons: {self._diagnose_autonomous_degradation(bn_rec)}. "
                            f"Human control required for safety.",
                urgency="high",
                allow_decline=False,
                timeout_seconds=None,
                reasons={
                    "auto_switch_to_human": True,
                    "auto_reliability": bn_rec.autonomous_reliability,
                    "threshold": self.LOW_THRESHOLD,
                    "diagnosis": self._diagnose_autonomous_degradation(bn_rec)
                }
            )
        
        return None
    
    def _apply_phase_rules(
        self,
        current_mode: ControlMode,
        recommended_mode: ControlMode,
        risk_level: PhaseRiskLevel,
        mission_phase: str,
        bn_rec: ModeRecommendation
    ) -> AuthorityDecision:
        """Apply phase-dependent authority rules"""
        
        # No change needed
        if current_mode == recommended_mode:
            return AuthorityDecision(
                action_type=ActionType.NONE,
                target_mode=current_mode,
                message=f"Mode stable: {current_mode.name}",
                explanation=f"Current mode is optimal for {mission_phase} phase.",
                urgency="low",
                allow_decline=False,
                timeout_seconds=None,
                reasons={"no_change_needed": True}
            )
        
        # ========================================
        # CRITICAL PHASES (Docking, DockingApproach)
        # ========================================
        if risk_level == PhaseRiskLevel.CRITICAL:
            return self._handle_critical_phase(current_mode, recommended_mode, mission_phase, bn_rec)
        
        # ========================================
        # HIGH RISK PHASES (Undocking)
        # ========================================
        elif risk_level == PhaseRiskLevel.HIGH:
            return self._handle_high_risk_phase(current_mode, recommended_mode, mission_phase, bn_rec)
        
        # ========================================
        # MEDIUM RISK PHASES (Inspection)
        # ========================================
        elif risk_level == PhaseRiskLevel.MEDIUM:
            return self._handle_medium_risk_phase(current_mode, recommended_mode, mission_phase, bn_rec)
        
        # ========================================
        # LOW RISK PHASES (Transit)
        # ========================================
        elif risk_level == PhaseRiskLevel.LOW:
            return self._handle_low_risk_phase(current_mode, recommended_mode, mission_phase, bn_rec)
        
        # ========================================
        # SAFE PHASES (Charging)
        # ========================================
        else:  # SAFE
            return self._handle_safe_phase(current_mode, recommended_mode, mission_phase, bn_rec)
    
    def _handle_critical_phase(
        self, 
        current_mode: ControlMode, 
        recommended_mode: ControlMode,
        mission_phase: str,
        bn_rec: ModeRecommendation
    ) -> AuthorityDecision:
        """Handle CRITICAL phase (Docking, DockingApproach)"""
        
        # Autonomous → Human: AUTO SWITCH (safety critical)
        if current_mode == ControlMode.AUTONOMOUS and recommended_mode == ControlMode.HUMAN:
            explanation = self._generate_explanation(
                "auto_to_human_critical",
                mission_phase,
                bn_rec,
                "Autonomous system detected conditions requiring human judgment"
            )
            
            return AuthorityDecision(
                action_type=ActionType.AUTO_SWITCH,
                target_mode=ControlMode.HUMAN,
                message=f"🚨 Critical Phase - Switching to Human Control",
                explanation=explanation,
                urgency="critical",
                allow_decline=False,
                timeout_seconds=None,
                reasons={
                    "phase": mission_phase,
                    "transition": "auto_to_human",
                    "auto_reliability": bn_rec.autonomous_reliability,
                    "human_reliability": bn_rec.human_reliability
                }
            )
        
        # Human → Autonomous: BLOCK (too risky)
        elif current_mode == ControlMode.HUMAN and recommended_mode == ControlMode.AUTONOMOUS:
            return AuthorityDecision(
                action_type=ActionType.BLOCK,
                target_mode=current_mode,
                message=f"❌ Autonomous mode blocked during {mission_phase}",
                explanation=f"Autonomous operation is not permitted during {mission_phase}. "
                            f"This critical phase requires human judgment and decision-making. "
                            f"Shared control is available if you need assistance with positioning or stabilization.",
                urgency="high",
                allow_decline=False,
                timeout_seconds=None,
                reasons={
                    "blocked": True,
                    "reason": "critical_phase_safety",
                    "phase": mission_phase
                }
            )
        
        # Any → Shared: SUGGEST
        elif recommended_mode == ControlMode.SHARED:
            explanation = self._generate_explanation(
                "suggest_shared_critical",
                mission_phase,
                bn_rec,
                "Shared control can assist during this critical phase"
            )
            
            return AuthorityDecision(
                action_type=ActionType.ASK,
                target_mode=ControlMode.SHARED,
                message=f"🤝 Shared control recommended for {mission_phase}",
                explanation=explanation,
                urgency="medium",
                allow_decline=True,
                timeout_seconds=30,
                reasons={
                    "phase": mission_phase,
                    "transition": f"{current_mode.name}_to_shared",
                    "benefits": "System can assist with stabilization while you focus on critical decisions"
                }
            )
        
        # Default: maintain current
        return AuthorityDecision(
            action_type=ActionType.NONE,
            target_mode=current_mode,
            message=f"Maintaining {current_mode.name} mode",
            explanation=f"Current mode is appropriate for {mission_phase}.",
            urgency="low",
            allow_decline=False,
            timeout_seconds=None,
            reasons={}
        )
    
    def _handle_high_risk_phase(
        self, 
        current_mode: ControlMode, 
        recommended_mode: ControlMode,
        mission_phase: str,
        bn_rec: ModeRecommendation
    ) -> AuthorityDecision:
        """Handle HIGH RISK phase (Undocking)"""
        
        # Autonomous → Human: AUTO SWITCH
        if current_mode == ControlMode.AUTONOMOUS and recommended_mode == ControlMode.HUMAN:
            explanation = self._generate_explanation(
                "auto_to_human_high",
                mission_phase,
                bn_rec,
                "Autonomous reliability has decreased"
            )
            
            return AuthorityDecision(
                action_type=ActionType.AUTO_SWITCH,
                target_mode=ControlMode.HUMAN,
                message=f"⚠️ High Risk Phase - Switching to Human Control",
                explanation=explanation,
                urgency="high",
                allow_decline=False,
                timeout_seconds=None,
                reasons={
                    "phase": mission_phase,
                    "auto_reliability": bn_rec.autonomous_reliability,
                    "human_reliability": bn_rec.human_reliability
                }
            )
        
        # Human → Autonomous: ASK (with detailed info)
        elif current_mode == ControlMode.HUMAN and recommended_mode == ControlMode.AUTONOMOUS:
            # Check if human is fatigued
            if bn_rec.human_reliability < self.LOW_THRESHOLD:
                urgency = "high"
                message_prefix = "⚠️ Your fatigue is high."
            else:
                urgency = "medium"
                message_prefix = ""
            
            explanation = self._generate_explanation(
                "human_to_auto_high",
                mission_phase,
                bn_rec,
                f"{message_prefix} Autonomous system can handle {mission_phase}"
            )
            
            return AuthorityDecision(
                action_type=ActionType.ASK,
                target_mode=ControlMode.AUTONOMOUS,
                message=f"{message_prefix} Switch to Autonomous for {mission_phase}?",
                explanation=explanation,
                urgency=urgency,
                allow_decline=True,
                timeout_seconds=45,
                reasons={
                    "phase": mission_phase,
                    "human_reliability": bn_rec.human_reliability,
                    "auto_reliability": bn_rec.autonomous_reliability,
                    "fatigue_warning": bn_rec.human_reliability < self.LOW_THRESHOLD
                }
            )
        
        # Any → Shared: SUGGEST
        elif recommended_mode == ControlMode.SHARED:
            explanation = self._generate_explanation(
                "suggest_shared",
                mission_phase,
                bn_rec,
                "Shared control balances human oversight with system assistance"
            )
            
            return AuthorityDecision(
                action_type=ActionType.SUGGEST,
                target_mode=ControlMode.SHARED,
                message=f"🤝 Shared control available for {mission_phase}",
                explanation=explanation,
                urgency="medium",
                allow_decline=True,
                timeout_seconds=30,
                reasons={"phase": mission_phase}
            )
        
        return AuthorityDecision(
            action_type=ActionType.NONE,
            target_mode=current_mode,
            message="Mode stable",
            explanation="",
            urgency="low",
            allow_decline=False,
            timeout_seconds=None,
            reasons={}
        )
    
    def _handle_medium_risk_phase(
        self, 
        current_mode: ControlMode, 
        recommended_mode: ControlMode,
        mission_phase: str,
        bn_rec: ModeRecommendation
    ) -> AuthorityDecision:
        """Handle MEDIUM RISK phase (Inspection)"""
        
        # Autonomous → Human: NOTIFY + SUGGEST
        if current_mode == ControlMode.AUTONOMOUS and recommended_mode == ControlMode.HUMAN:
            explanation = self._generate_explanation(
                "auto_degraded_medium",
                mission_phase,
                bn_rec,
                "Autonomous performance has decreased but can continue"
            )
            
            return AuthorityDecision(
                action_type=ActionType.ASK,
                target_mode=ControlMode.HUMAN,
                message="ℹ️ Autonomous performance degraded - Switch to manual?",
                explanation=explanation + " Autonomous mode can continue if you prefer.",
                urgency="medium",
                allow_decline=True,
                timeout_seconds=60,
                reasons={
                    "phase": mission_phase,
                    "auto_reliability": bn_rec.autonomous_reliability,
                    "can_continue_auto": True
                }
            )
        
        # Human → Autonomous: SUGGEST
        elif current_mode == ControlMode.HUMAN and recommended_mode == ControlMode.AUTONOMOUS:
            explanation = self._generate_explanation(
                "offer_auto_assist",
                mission_phase,
                bn_rec,
                "Autopilot can handle routine inspection tasks"
            )
            
            return AuthorityDecision(
                action_type=ActionType.SUGGEST,
                target_mode=ControlMode.AUTONOMOUS,
                message="🤖 Autopilot available - Enable to conserve attention?",
                explanation=explanation + " You can monitor without active control.",
                urgency="low",
                allow_decline=True,
                timeout_seconds=None,
                reasons={
                    "phase": mission_phase,
                    "benefit": "reduce_operator_workload",
                    "auto_reliability": bn_rec.autonomous_reliability
                }
            )
        
        # Any → Shared
        elif recommended_mode == ControlMode.SHARED:
            explanation = self._generate_explanation(
                "suggest_shared",
                mission_phase,
                bn_rec,
                "Shared control optimizes human-system performance"
            )
            
            return AuthorityDecision(
                action_type=ActionType.SUGGEST,
                target_mode=ControlMode.SHARED,
                message="🤝 Shared control recommended",
                explanation=explanation,
                urgency="low",
                allow_decline=True,
                timeout_seconds=None,
                reasons={"phase": mission_phase}
            )
        
        return AuthorityDecision(
            action_type=ActionType.NONE,
            target_mode=current_mode,
            message="",
            explanation="",
            urgency="low",
            allow_decline=False,
            timeout_seconds=None,
            reasons={}
        )
    
    def _handle_low_risk_phase(
        self, 
        current_mode: ControlMode, 
        recommended_mode: ControlMode,
        mission_phase: str,
        bn_rec: ModeRecommendation
    ) -> AuthorityDecision:
        """Handle LOW RISK phase (Transit)"""
        
        # Autonomous → Human: NOTIFY only
        if current_mode == ControlMode.AUTONOMOUS and recommended_mode == ControlMode.HUMAN:
            explanation = self._generate_explanation(
                "auto_degraded_low",
                mission_phase,
                bn_rec,
                "Autonomous performance decreased"
            )
            
            return AuthorityDecision(
                action_type=ActionType.NOTIFY,
                target_mode=current_mode,
                message="ℹ️ Autonomous performance note - Available to take control",
                explanation=explanation + " No immediate action required during transit.",
                urgency="low",
                allow_decline=False,
                timeout_seconds=None,
                reasons={
                    "phase": mission_phase,
                    "auto_reliability": bn_rec.autonomous_reliability,
                    "info_only": True
                }
            )
        
        # Human → Autonomous: SUGGEST (help operator rest)
        elif current_mode == ControlMode.HUMAN and recommended_mode == ControlMode.AUTONOMOUS:
            explanation = self._generate_explanation(
                "offer_auto_rest",
                mission_phase,
                bn_rec,
                "Autopilot can handle transit to conserve your attention"
            )
            
            return AuthorityDecision(
                action_type=ActionType.SUGGEST,
                target_mode=ControlMode.AUTONOMOUS,
                message="🤖 Autopilot available for transit - Rest and monitor?",
                explanation=explanation + " This allows you to conserve attention for more demanding phases ahead.",
                urgency="low",
                allow_decline=True,
                timeout_seconds=None,
                reasons={
                    "phase": mission_phase,
                    "benefit": "operator_rest",
                    "human_reliability": bn_rec.human_reliability
                }
            )
        
        return AuthorityDecision(
            action_type=ActionType.SUGGEST if recommended_mode != current_mode else ActionType.NONE,
            target_mode=recommended_mode,
            message=f"Consider {recommended_mode.name} mode?",
            explanation=f"{recommended_mode.name} mode is available during low-risk {mission_phase}.",
            urgency="low",
            allow_decline=True,
            timeout_seconds=None,
            reasons={"phase": mission_phase}
        )
    
    def _handle_safe_phase(
        self, 
        current_mode: ControlMode, 
        recommended_mode: ControlMode,
        mission_phase: str,
        bn_rec: ModeRecommendation
    ) -> AuthorityDecision:
        """Handle SAFE phase (Charging)"""
        
        # During charging, autonomous is preferred
        if recommended_mode == ControlMode.AUTONOMOUS and current_mode != ControlMode.AUTONOMOUS:
            return AuthorityDecision(
                action_type=ActionType.AUTO_SWITCH,
                target_mode=ControlMode.AUTONOMOUS,
                message=f"✓ Switching to Autonomous mode during {mission_phase}",
                explanation=f"ROV is safely docked and charging. Autonomous mode will monitor systems. "
                            f"You can take control at any time if needed.",
                urgency="low",
                allow_decline=False,
                timeout_seconds=None,
                reasons={
                    "phase": mission_phase,
                    "safe_phase_auto": True
                }
            )
        
        return AuthorityDecision(
            action_type=ActionType.NONE,
            target_mode=current_mode,
            message="System monitoring during charging",
            explanation="",
            urgency="low",
            allow_decline=False,
            timeout_seconds=None,
            reasons={"phase": mission_phase}
        )
    
    def _generate_explanation(
        self,
        scenario: str,
        mission_phase: str,
        bn_rec: ModeRecommendation,
        summary: str
    ) -> str:
        """
        Generate detailed human-readable explanation for mode recommendation
        
        This builds trust by showing WHY the system is making a recommendation
        """
        explanation_parts = [summary + "."]
        
        # Add reliability comparison
        explanation_parts.append(
            f"\n\n📊 Performance Assessment:"
        )
        explanation_parts.append(
            f"\n  • Your reliability: {bn_rec.human_reliability:.0%} "
            f"({self._reliability_label(bn_rec.human_reliability)})"
        )
        explanation_parts.append(
            f"\n  • System reliability: {bn_rec.autonomous_reliability:.0%} "
            f"({self._reliability_label(bn_rec.autonomous_reliability)})"
        )
        
        if bn_rec.docking_reliability is not None:
            explanation_parts.append(
                f"\n  • Docking reliability: {bn_rec.docking_reliability:.0%}"
            )
        
        # Add specific factors based on scenario
        if "auto_to_human" in scenario:
            factors = self._diagnose_autonomous_degradation(bn_rec)
            if factors:
                explanation_parts.append("\n\n🔍 Autonomous System Status:")
                for factor in factors:
                    explanation_parts.append(f"\n  • {factor}")
        
        elif "human" in scenario and bn_rec.human_reliability < self.LOW_THRESHOLD:
            explanation_parts.append("\n\n⚠️ Your Current State:")
            if bn_rec.human_reliability < self.CRITICAL_LOW_THRESHOLD:
                explanation_parts.append("\n  • Fatigue level is critically high")
                explanation_parts.append("\n  • Attention capacity is significantly reduced")
                explanation_parts.append("\n  • Cognitive workload may be elevated")
            else:
                explanation_parts.append("\n  • Fatigue is accumulating")
                explanation_parts.append("\n  • Consider using automation to reduce workload")
        
        # Add phase-specific context
        explanation_parts.append(f"\n\n📍 Mission Context: {mission_phase} phase")
        
        # Add confidence level
        confidence_desc = "very confident" if bn_rec.confidence > 0.8 else \
                         "confident" if bn_rec.confidence > 0.6 else \
                         "moderately confident"
        explanation_parts.append(
            f"\n\nSystem is {confidence_desc} in this recommendation ({bn_rec.confidence:.0%})."
        )
        
        return "".join(explanation_parts)
    
    def _reliability_label(self, reliability: float) -> str:
        """Convert reliability score to human-readable label"""
        if reliability >= self.HIGH_THRESHOLD:
            return "Excellent"
        elif reliability >= 0.7:
            return "Good"
        elif reliability >= self.LOW_THRESHOLD:
            return "Fair"
        elif reliability >= self.CRITICAL_LOW_THRESHOLD:
            return "Low"
        else:
            return "Critical"
    
    def _diagnose_autonomous_degradation(self, bn_rec: ModeRecommendation) -> list:
        """
        Diagnose reasons for autonomous performance degradation
        
        In full implementation, this would query the BN for:
        - Navigation reliability factors
        - Perception reliability factors
        - Control stability factors
        """
        factors = []
        
        if bn_rec.autonomous_reliability < 0.6:
            # In real implementation, query actual BN nodes
            # For now, provide plausible reasons
            factors.append("Navigation accuracy reduced (possible USBL degradation)")
            factors.append("Environmental conditions affecting stability")
        
        if bn_rec.docking_reliability is not None and bn_rec.docking_reliability < 0.7:
            factors.append(f"Docking conditions suboptimal (reliability: {bn_rec.docking_reliability:.0%})")
        
        if not factors:
            factors.append("Minor performance degradation detected")
        
        return factors