#!/usr/bin/env python3
"""
Standalone Human State Simulator (No ROS2)
Simulates operator cognitive states over time
"""

import numpy as np
import pandas as pd
import time
from dataclasses import dataclass
from typing import Dict
import yaml


@dataclass
class ModelParameters:
    """Temporal model parameters"""
    k_fatigue: float = 0.008
    k_stress_gen: float = 0.02
    k_stress_decay: float = 0.03
    beta_attention: float = 0.98
    alpha_fatigue: float = 0.3
    alpha_stress: float = 0.2


class OperatorState:
    """Represents cognitive state of a single operator"""
    
    def __init__(self, operator_id: str):
        self.operator_id = operator_id
        self.state = {
            'fatigue': 0.1,
            'stress': 0.2,
            'attention': 0.9,
            'work_time': 0.0,
            'situational_awareness': 1.0
        }
        self.is_active = False
        self.shifts_worked = 0


class HumanStateSimulatorStandalone:
    """Standalone human state simulator"""
    
    def __init__(self, config_file='human_state_params.yaml'):
        self.params = ModelParameters()
        self.sim_time_hours = 0.0
        
        # Initialize operators
        self.operators = {
            'A': OperatorState('A'),
            'B': OperatorState('B')
        }
        self.active_operator_id = 'A'
        self.operators['A'].is_active = True
        
        # Handover state
        self.in_handover = False
        self.handover_start_time = None
        self.handover_duration = 0.5  # 30 minutes
        
        # History for logging
        self.history = []
        
        print("="*70)
        print("HUMAN STATE SIMULATOR - STANDALONE MODE")
        print("="*70)
        print(f"Operators initialized: {list(self.operators.keys())}")
        print(f"Active operator: {self.active_operator_id}")
        print("="*70)
    
    def update(self, delta_t: float, task_complexity: float, workload: float):
        """Update operator states"""
        
        self.sim_time_hours += delta_t
        
        # Update active operators
        for op in self.operators.values():
            if op.is_active:
                self._update_operator_state(op, delta_t, task_complexity, workload)
        
        # Update handover progress
        if self.in_handover:
            self._update_handover()
        
        # Log current state
        self._log_state(task_complexity, workload)
    
    def _update_operator_state(self, operator, delta_t, complexity, workload):
        """Update single operator's state"""
        
        state = operator.state
        
        # Fatigue
        fatigue_rate = self.params.k_fatigue * (1 + complexity)
        state['fatigue'] += fatigue_rate * delta_t
        state['fatigue'] = float(np.clip(state['fatigue'], 0, 1))
        
        # Stress
        stress_increase = self.params.k_stress_gen * workload
        stress_decay = self.params.k_stress_decay * state['stress']
        state['stress'] += (stress_increase - stress_decay) * delta_t
        
        # Handover stress
        if self.in_handover and operator.operator_id == self.handover_from_operator:
            state['stress'] += 0.1 * delta_t
        
        state['stress'] = float(np.clip(state['stress'], 0, 1))
        
        # Attention
        attention_factor = 1 - (
            self.params.alpha_fatigue * state['fatigue'] + 
            self.params.alpha_stress * state['stress']
        )
        state['attention'] *= (self.params.beta_attention ** delta_t) * max(attention_factor, 0.0)
        state['attention'] *= state['situational_awareness']
        state['attention'] = float(np.clip(state['attention'], 0.2, 1.0))
        
        # Work time
        state['work_time'] += delta_t
    
    def initiate_handover(self, from_op_id: str, to_op_id: str):
        """Start handover between operators"""
        
        print("\n" + "="*70)
        print(f"HANDOVER INITIATED: {from_op_id} → {to_op_id}")
        print(f"Time: {self.sim_time_hours:.2f} sim-hours")
        print("="*70)
        
        self.in_handover = True
        self.handover_start_time = self.sim_time_hours
        self.handover_from_operator = from_op_id
        self.handover_to_operator = to_op_id
        
        # Incoming operator starts fresh
        incoming = self.operators[to_op_id]
        incoming.state['fatigue'] = 0.1
        incoming.state['stress'] = 0.3
        incoming.state['attention'] = 0.9
        incoming.state['situational_awareness'] = 0.4
        incoming.state['work_time'] = 0.0
        incoming.is_active = True
    
    def _update_handover(self):
        """Update handover progress"""
        
        elapsed = self.sim_time_hours - self.handover_start_time
        progress = elapsed / self.handover_duration
        
        if progress >= 1.0:
            self._complete_handover()
        else:
            # SA ramp-up
            incoming = self.operators[self.handover_to_operator]
            incoming.state['situational_awareness'] = 0.4 + (0.9 - 0.4) * progress
    
    def _complete_handover(self):
        """Complete handover"""
        
        print("\n" + "="*70)
        print("HANDOVER COMPLETE")
        print(f"Time: {self.sim_time_hours:.2f} sim-hours")
        print("="*70)
        
        # Departing operator goes off-duty
        departing = self.operators[self.handover_from_operator]
        departing.is_active = False
        departing.shifts_worked += 1
        
        print(f"Operator {self.handover_from_operator} final state:")
        print(f"  Fatigue: {departing.state['fatigue']:.3f}")
        print(f"  Stress: {departing.state['stress']:.3f}")
        print(f"  Attention: {departing.state['attention']:.3f}")
        
        # Incoming takes control
        incoming = self.operators[self.handover_to_operator]
        incoming.state['situational_awareness'] = 0.9
        incoming.state['stress'] = 0.2
        
        self.active_operator_id = self.handover_to_operator
        self.in_handover = False
        
        print(f"Operator {self.active_operator_id} now in control")
        print("="*70)
    
    def _log_state(self, complexity, workload):
        """Log current state"""
        
        active_op = self.operators[self.active_operator_id]
        
        self.history.append({
            'sim_time_hours': self.sim_time_hours,
            'active_operator': self.active_operator_id,
            'in_handover': self.in_handover,
            'task_complexity': complexity,
            'workload': workload,
            'fatigue': active_op.state['fatigue'],
            'stress': active_op.state['stress'],
            'attention': active_op.state['attention'],
            'situational_awareness': active_op.state['situational_awareness'],
            'work_time': active_op.state['work_time']
        })
    
    def get_active_state(self):
        """Get current active operator state"""
        active_op = self.operators[self.active_operator_id]
        return {
            'operator_id': self.active_operator_id,
            'in_handover': self.in_handover,
            **active_op.state
        }
    
    def save_results(self, filename='human_state_results.csv'):
        """Save results to CSV"""
        df = pd.DataFrame(self.history)
        df.to_csv(filename, index=False)
        print(f"\n✓ Results saved to {filename}")
        return df


def main():
    """Test standalone simulator"""
    
    simulator = HumanStateSimulator()
    
    # Simple 2-hour test
    delta_t = 0.1  # 6-minute steps
    
    for i in range(20):  # 2 hours
        time_hours = i * delta_t
        
        # Varying task load
        if time_hours < 1:
            complexity = 0.3
            workload = 0.3
        else:
            complexity = 0.7
            workload = 0.8
        
        simulator.update(delta_t, complexity, workload)
        
        # Print every 30 minutes
        if i % 5 == 0:
            state = simulator.get_active_state()
            print(f"\nTime: {time_hours:.1f}h | "
                  f"F: {state['fatigue']:.3f} | "
                  f"S: {state['stress']:.3f} | "
                  f"A: {state['attention']:.3f}")
    
    simulator.save_results()


if __name__ == '__main__':
    main()