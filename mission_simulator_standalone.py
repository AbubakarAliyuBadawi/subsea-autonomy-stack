#!/usr/bin/env python3
"""
Complete Mission Simulator - Standalone (No ROS2)
Orchestrates 12-hour mission with operator shifts
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from human_state_simulator_standalone import HumanStateSimulatorStandalone
import time


class MissionSimulator:
    """Complete mission simulation orchestrator"""
    
    def __init__(self):
        self.human_sim = HumanStateSimulatorStandalone()
        self.mission_duration = 12.0  # hours
        self.shift_duration = 6.0
        self.time_step = 0.1  # 6-minute steps
        
        # Mission timeline
        self.timeline = self._create_mission_timeline()
        
        print("\n" + "="*70)
        print("MISSION SIMULATOR INITIALIZED")
        print("="*70)
        print(f"Duration: {self.mission_duration} sim-hours")
        print(f"Shift duration: {self.shift_duration} hours")
        print(f"Timeline events: {len(self.timeline)}")
        print("="*70)
    
    def _create_mission_timeline(self):
        """Create mission timeline"""
        return [
            # Operator A shift (0-6h)
            {'time': 0.0, 'phase': 'Transit', 'complexity': 0.3, 'workload': 0.3},
            {'time': 1.0, 'phase': 'Inspection', 'complexity': 0.6, 'workload': 0.6},
            {'time': 3.0, 'phase': 'Inspection', 'complexity': 0.7, 'workload': 0.8},
            {'time': 4.5, 'phase': 'Transit', 'complexity': 0.4, 'workload': 0.4},
            {'time': 5.5, 'phase': 'DockingApproach', 'complexity': 0.9, 'workload': 0.9},
            
            # Shift change at 6.0h
            
            # Operator B shift (6-12h)
            {'time': 6.5, 'phase': 'Undocking', 'complexity': 0.8, 'workload': 0.8},
            {'time': 7.0, 'phase': 'Transit', 'complexity': 0.4, 'workload': 0.4},
            {'time': 8.0, 'phase': 'Inspection', 'complexity': 0.7, 'workload': 0.8},
            {'time': 10.0, 'phase': 'Inspection', 'complexity': 0.8, 'workload': 0.9},
            {'time': 11.0, 'phase': 'DockingApproach', 'complexity': 0.9, 'workload': 0.95},
            {'time': 11.5, 'phase': 'Docking', 'complexity': 0.95, 'workload': 0.95},
            {'time': 12.0, 'phase': 'Charging', 'complexity': 0.1, 'workload': 0.1}
        ]
    
    def _get_current_task(self, current_time):
        """Get current task parameters from timeline"""
        
        # Find current event
        current_event = self.timeline[0]
        for event in self.timeline:
            if current_time >= event['time']:
                current_event = event
            else:
                break
        
        return current_event['complexity'], current_event['workload'], current_event['phase']
    
    def run(self):
        """Run complete 12-hour mission simulation"""
        
        print("\n" + "="*70)
        print("STARTING 12-HOUR MISSION SIMULATION")
        print("="*70)
        
        start_real_time = time.time()
        
        num_steps = int(self.mission_duration / self.time_step)
        shift_change_triggered = False
        
        for step in range(num_steps):
            current_time = step * self.time_step
            
            # Get current task
            complexity, workload, phase = self._get_current_task(current_time)
            
            # Check for shift change (at 5.75h - 15min before 6h)
            if current_time >= 5.75 and not shift_change_triggered:
                self.human_sim.initiate_handover('A', 'B')
                shift_change_triggered = True
            
            # Update human state
            self.human_sim.update(self.time_step, complexity, workload)
            
            # Progress report every sim-hour
            if step % 10 == 0:
                state = self.human_sim.get_active_state()
                print(f"\n⏱️  {current_time:5.1f}h | Phase: {phase:15s} | "
                      f"Op: {state['operator_id']} | "
                      f"F: {state['fatigue']:.3f} | "
                      f"S: {state['stress']:.3f} | "
                      f"A: {state['attention']:.3f}")
        
        elapsed_real = time.time() - start_real_time
        
        print("\n" + "="*70)
        print("🎉 MISSION COMPLETE!")
        print("="*70)
        print(f"Simulation completed in {elapsed_real:.2f} seconds")
        print("="*70)
        
        # Save and plot results
        df = self.human_sim.save_results('mission_results.csv')
        self.plot_results(df)
        
        return df
    
    def plot_results(self, df):
        """Plot simulation results"""
        
        print("\nGenerating plots...")
        
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        
        # Plot 1: Human states
        axes[0].plot(df['sim_time_hours'], df['fatigue'], 
                    label='Fatigue', linewidth=2, color='#e74c3c')
        axes[0].plot(df['sim_time_hours'], df['stress'], 
                    label='Stress', linewidth=2, color='#f39c12')
        axes[0].plot(df['sim_time_hours'], df['attention'], 
                    label='Attention', linewidth=2, color='#2ecc71')
        axes[0].axvline(x=6.0, color='purple', linestyle='--', linewidth=2, 
                       label='Shift Change')
        axes[0].set_ylabel('State Value', fontsize=12, fontweight='bold')
        axes[0].set_title('Human Cognitive States Over 12-Hour Mission', 
                         fontsize=14, fontweight='bold')
        axes[0].legend(loc='best')
        axes[0].grid(True, alpha=0.3)
        axes[0].set_ylim([0, 1])
        
        # Plot 2: Task parameters
        axes[1].plot(df['sim_time_hours'], df['task_complexity'], 
                    label='Complexity', linewidth=2, linestyle='--', color='#9b59b6')
        axes[1].plot(df['sim_time_hours'], df['workload'], 
                    label='Workload', linewidth=2, linestyle='--', color='#3498db')
        axes[1].axvline(x=6.0, color='purple', linestyle='--', linewidth=2)
        axes[1].set_ylabel('Task Demand', fontsize=12, fontweight='bold')
        axes[1].set_title('Mission Task Profile', fontsize=14, fontweight='bold')
        axes[1].legend(loc='best')
        axes[1].grid(True, alpha=0.3)
        axes[1].set_ylim([0, 1])
        
        # Plot 3: Situational Awareness
        axes[2].plot(df['sim_time_hours'], df['situational_awareness'], 
                    label='Situational Awareness', linewidth=2.5, color='#1abc9c')
        axes[2].axvline(x=6.0, color='purple', linestyle='--', linewidth=2, 
                       label='Shift Change')
        axes[2].fill_between(df['sim_time_hours'], 0, 1, 
                            where=df['in_handover'], 
                            alpha=0.2, color='orange', label='Handover Period')
        axes[2].set_xlabel('Time (hours)', fontsize=12, fontweight='bold')
        axes[2].set_ylabel('SA Level', fontsize=12, fontweight='bold')
        axes[2].set_title('Situational Awareness (Shows Shift Transition)', 
                         fontsize=14, fontweight='bold')
        axes[2].legend(loc='best')
        axes[2].grid(True, alpha=0.3)
        axes[2].set_ylim([0, 1])
        
        plt.tight_layout()
        plt.savefig('mission_simulation_results.png', dpi=300, bbox_inches='tight')
        print("✓ Plot saved: mission_simulation_results.png")
        plt.show()


def main():
    """Run complete mission simulation"""
    
    simulator = MissionSimulator()
    results = simulator.run()
    
    # Print summary statistics
    print("\n" + "="*70)
    print("SUMMARY STATISTICS")
    print("="*70)
    
    # Split by operator
    op_a_data = results[results['active_operator'] == 'A']
    op_b_data = results[results['active_operator'] == 'B']
    
    print("\nOperator A (0-6h):")
    print(f"  Final Fatigue: {op_a_data['fatigue'].iloc[-1]:.3f}")
    print(f"  Final Stress: {op_a_data['stress'].iloc[-1]:.3f}")
    print(f"  Final Attention: {op_a_data['attention'].iloc[-1]:.3f}")
    
    print("\nOperator B (6-12h):")
    print(f"  Final Fatigue: {op_b_data['fatigue'].iloc[-1]:.3f}")
    print(f"  Final Stress: {op_b_data['stress'].iloc[-1]:.3f}")
    print(f"  Final Attention: {op_b_data['attention'].iloc[-1]:.3f}")
    
    print("\n" + "="*70)


if __name__ == '__main__':
    main()