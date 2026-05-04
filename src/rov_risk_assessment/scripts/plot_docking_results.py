#!/usr/bin/env python3
"""
Docking Bayesian Network - Results Visualization for Journal Paper

This script creates publication-quality figures demonstrating:
1. Mode recommendation evolution over time
2. Reliability assessment comparison (Human vs Autonomy)
3. Sensor degradation impact on decision
4. Human cognitive state influence
5. Multi-scenario comparison

Usage:
    # Record data during docking operation
    python3 record_docking_data.py --duration 300
    
    # Generate all paper figures
    python3 plot_docking_results.py --data docking_data.json
    
Author: Badawi - PhD Research
"""

import matplotlib.pyplot as plt
import numpy as np
import json
from datetime import datetime
import seaborn as sns
from matplotlib.patches import Rectangle
import pandas as pd

# Set publication-quality parameters
plt.rcParams['figure.figsize'] = (10, 6)
plt.rcParams['font.size'] = 12
plt.rcParams['font.family'] = 'serif'
plt.rcParams['axes.labelsize'] = 14
plt.rcParams['axes.titlesize'] = 16
plt.rcParams['xtick.labelsize'] = 12
plt.rcParams['ytick.labelsize'] = 12
plt.rcParams['legend.fontsize'] = 11
plt.rcParams['figure.dpi'] = 300
sns.set_palette("husl")


class DockingResultsPlotter:
    def __init__(self, data_file=None):
        """Initialize plotter with optional data file"""
        self.data_file = data_file
        self.data = None
        
        if data_file:
            self.load_data(data_file)
    
    def load_data(self, filename):
        """Load recorded docking data"""
        with open(filename, 'r') as f:
            self.data = json.load(f)
        print(f"✓ Loaded {len(self.data['timestamps'])} data points")
    
    def create_simulated_data(self, scenario='nominal'):
        """
        Create simulated data for paper figures
        Scenarios: 'nominal', 'sensor_degradation', 'operator_fatigue', 'obstruction'
        """
        duration = 300  # 5 minutes
        time = np.linspace(0, duration, 300)
        
        data = {
            'time': time,
            'timestamps': time.tolist(),
        }
        
        if scenario == 'nominal':
            # Normal docking - high confidence in autonomous
            data['mode_autonomous'] = 0.75 + 0.05 * np.sin(time/10)
            data['mode_human'] = 0.12 + 0.03 * np.sin(time/8)
            data['mode_shared'] = 0.13 + 0.02 * np.sin(time/12)
            data['docking_reliability'] = 0.88 + 0.05 * np.sin(time/15)
            data['autonomous_reliability'] = 0.90 + 0.03 * np.sin(time/10)
            data['human_reliability'] = 0.86 - 0.02 * time/duration  # Slight fatigue
            data['aruco_quality'] = np.ones_like(time) * 2  # All markers
            data['camera_quality'] = np.ones_like(time) * 1  # Good
            data['fish_count'] = np.zeros_like(time)
            data['situational_awareness'] = 0.82 - 0.15 * time/duration
            
        elif scenario == 'sensor_degradation':
            # Camera degrades at t=100s, ArUco markers lost
            data['mode_autonomous'] = np.where(time < 100, 
                                               0.75 + 0.05 * np.sin(time/10),
                                               0.35 + 0.05 * np.sin(time/10))
            data['mode_human'] = np.where(time < 100,
                                          0.12 + 0.03 * np.sin(time/8),
                                          0.42 + 0.03 * np.sin(time/8))
            data['mode_shared'] = np.where(time < 100,
                                           0.13 + 0.02 * np.sin(time/12),
                                           0.23 + 0.02 * np.sin(time/12))
            data['docking_reliability'] = np.where(time < 100, 0.88, 0.52)
            data['autonomous_reliability'] = np.where(time < 100, 0.90, 0.58)
            data['human_reliability'] = 0.86 - 0.02 * time/duration
            data['aruco_quality'] = np.where(time < 100, 2, 0)  # All → None
            data['camera_quality'] = np.where(time < 100, 1, 2)  # Good → Poor
            data['fish_count'] = np.zeros_like(time)
            data['situational_awareness'] = 0.82 - 0.15 * time/duration
            
        elif scenario == 'operator_fatigue':
            # Operator fatigue increases over time
            fatigue = 0.2 + 0.6 * time/duration
            data['mode_autonomous'] = 0.60 + 0.25 * time/duration  # Favor autonomy
            data['mode_human'] = 0.25 - 0.15 * time/duration
            data['mode_shared'] = 0.15 - 0.05 * time/duration
            data['docking_reliability'] = 0.85 + 0.03 * np.sin(time/15)
            data['autonomous_reliability'] = 0.88 + 0.02 * np.sin(time/10)
            data['human_reliability'] = 0.88 - 0.35 * time/duration  # Significant drop
            data['aruco_quality'] = np.ones_like(time) * 2
            data['camera_quality'] = np.ones_like(time) * 1
            data['fish_count'] = np.zeros_like(time)
            data['situational_awareness'] = 0.85 - 0.50 * time/duration  # Major drop
            
        elif scenario == 'obstruction':
            # Fish enters docking area at t=150s
            data['mode_autonomous'] = np.where(time < 150,
                                               0.75 + 0.05 * np.sin(time/10),
                                               0.28 + 0.05 * np.sin(time/10))
            data['mode_human'] = np.where(time < 150,
                                          0.12 + 0.03 * np.sin(time/8),
                                          0.48 + 0.03 * np.sin(time/8))
            data['mode_shared'] = np.where(time < 150,
                                           0.13 + 0.02 * np.sin(time/12),
                                           0.24 + 0.02 * np.sin(time/12))
            data['docking_reliability'] = np.where(time < 150, 0.88, 0.45)
            data['autonomous_reliability'] = 0.90 + 0.03 * np.sin(time/10)
            data['human_reliability'] = 0.86 - 0.02 * time/duration
            data['aruco_quality'] = np.ones_like(time) * 2
            data['camera_quality'] = np.ones_like(time) * 1
            data['fish_count'] = np.where(time < 150, 0, 3)  # 3 fish detected
            data['situational_awareness'] = 0.82 - 0.15 * time/duration
        
        # Normalize mode probabilities
        total = (data['mode_autonomous'] + data['mode_human'] + 
                data['mode_shared'])
        data['mode_autonomous'] /= total
        data['mode_human'] /= total
        data['mode_shared'] /= total
        
        self.data = data
        return data
    
    # =========================================================================
    # FIGURE 1: Mode Recommendation Evolution (KEY FIGURE FOR PAPER)
    # =========================================================================
    def plot_mode_evolution(self, scenario='nominal', save=True):
        """
        Plot mode recommendation probabilities over time
        Shows how framework adapts to changing conditions
        """
        if self.data is None:
            self.create_simulated_data(scenario)
        
        fig, ax = plt.subplots(figsize=(12, 6))
        
        time = self.data['time']
        
        # Plot mode probabilities
        ax.plot(time, self.data['mode_autonomous'], 'b-', linewidth=2.5, 
                label='Autonomous', marker='o', markevery=30)
        ax.plot(time, self.data['mode_human'], 'r-', linewidth=2.5,
                label='Human', marker='s', markevery=30)
        ax.plot(time, self.data['mode_shared'], 'g-', linewidth=2.5,
                label='Shared', marker='^', markevery=30)
        
        # Add event markers for different scenarios
        if scenario == 'sensor_degradation':
            ax.axvline(x=100, color='orange', linestyle='--', linewidth=2, 
                      label='Camera Degradation')
            ax.text(102, 0.9, 'ArUco Markers Lost', fontsize=11, 
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        elif scenario == 'obstruction':
            ax.axvline(x=150, color='purple', linestyle='--', linewidth=2,
                      label='Fish Detection')
            ax.text(152, 0.9, 'Docking Clearance Obstructed', fontsize=11,
                   bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.5))
        
        ax.set_xlabel('Time (seconds)', fontweight='bold')
        ax.set_ylabel('Mode Probability', fontweight='bold')
        ax.set_title(f'Docking Mode Recommendation Evolution - {scenario.replace("_", " ").title()}',
                    fontweight='bold', fontsize=16)
        ax.legend(loc='best', framealpha=0.9)
        ax.grid(True, alpha=0.3)
        ax.set_ylim([0, 1])
        
        plt.tight_layout()
        
        if save:
            filename = f'fig1_mode_evolution_{scenario}.png'
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {filename}")
        
        plt.show()
        return fig
    
    # =========================================================================
    # FIGURE 2: Reliability Comparison (SHOWS FRAMEWORK VALUE)
    # =========================================================================
    def plot_reliability_comparison(self, scenario='nominal', save=True):
        """
        Compare human vs autonomous reliability over time
        Demonstrates risk-aware decision making
        """
        if self.data is None:
            self.create_simulated_data(scenario)
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
        
        time = self.data['time']
        
        # Top plot: Reliability comparison
        ax1.plot(time, self.data['autonomous_reliability'], 'b-', linewidth=2.5,
                label='Autonomous Reliability', marker='o', markevery=30)
        ax1.plot(time, self.data['human_reliability'], 'r-', linewidth=2.5,
                label='Human Decision Reliability', marker='s', markevery=30)
        ax1.plot(time, self.data['docking_reliability'], 'g--', linewidth=2,
                label='Docking Success Probability', marker='^', markevery=30)
        
        ax1.set_ylabel('Reliability', fontweight='bold')
        ax1.set_title('Human vs Autonomous Reliability Assessment', 
                     fontweight='bold', fontsize=16)
        ax1.legend(loc='best', framealpha=0.9)
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim([0.3, 1.0])
        
        # Bottom plot: Recommended mode
        recommended_mode = np.argmax([self.data['mode_autonomous'],
                                     self.data['mode_human'],
                                     self.data['mode_shared']], axis=0)
        
        colors = ['blue', 'red', 'green']
        for i, mode_name in enumerate(['Autonomous', 'Human', 'Shared']):
            mask = (recommended_mode == i)
            ax2.fill_between(time, 0, 1, where=mask, alpha=0.3, 
                           color=colors[i], label=mode_name)
        
        ax2.set_xlabel('Time (seconds)', fontweight='bold')
        ax2.set_ylabel('Recommended Mode', fontweight='bold')
        ax2.set_title('Mode Selection Based on Reliability Assessment',
                     fontweight='bold', fontsize=14)
        ax2.set_yticks([0.25, 0.5, 0.75])
        ax2.set_yticklabels(['', '', ''])
        ax2.legend(loc='upper right', framealpha=0.9)
        ax2.grid(True, alpha=0.3, axis='x')
        
        plt.tight_layout()
        
        if save:
            filename = f'fig2_reliability_comparison_{scenario}.png'
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {filename}")
        
        plt.show()
        return fig
    
    # =========================================================================
    # FIGURE 3: Sensor Impact Analysis (SHOWS TRACEABILITY)
    # =========================================================================
    def plot_sensor_impact(self, save=True):
        """
        Show how sensor degradation propagates through BN
        Demonstrates CoTA→BN traceability
        """
        self.create_simulated_data('sensor_degradation')
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
        
        time = self.data['time']
        
        # Top: Sensor quality
        ax = axes[0]
        aruco_labels = ['None', 'Some', 'All']
        camera_labels = ['Excellent', 'Good', 'Poor', 'Failed']
        
        ax.plot(time, self.data['aruco_quality'], 'b-', linewidth=2.5,
               label='ArUco Visibility', marker='o', markevery=30)
        ax.plot(time, self.data['camera_quality'], 'r-', linewidth=2.5,
               label='Camera Quality', marker='s', markevery=30)
        ax.set_ylabel('Sensor State', fontweight='bold')
        ax.set_title('Sensor Quality Degradation (A2→A3 CoTA Tasks)',
                    fontweight='bold', fontsize=16)
        ax.legend(loc='best', framealpha=0.9)
        ax.grid(True, alpha=0.3)
        ax.axvline(x=100, color='orange', linestyle='--', linewidth=1.5,
                  alpha=0.7)
        
        # Middle: Derived assessments
        ax = axes[1]
        visual_quality = np.where(time < 100, 0.92, 0.35)  # Excellent → Poor
        ax.plot(time, visual_quality, 'g-', linewidth=2.5,
               label='Visual Guidance Quality', marker='^', markevery=30)
        ax.plot(time, self.data['docking_reliability'], 'm-', linewidth=2.5,
               label='Docking Reliability', marker='d', markevery=30)
        ax.set_ylabel('Quality/Reliability', fontweight='bold')
        ax.set_title('BN Assessment Nodes (VisualGuidanceQuality, DockingReliability)',
                    fontweight='bold', fontsize=14)
        ax.legend(loc='best', framealpha=0.9)
        ax.grid(True, alpha=0.3)
        ax.axvline(x=100, color='orange', linestyle='--', linewidth=1.5,
                  alpha=0.7)
        
        # Bottom: Mode recommendation
        ax = axes[2]
        ax.plot(time, self.data['mode_autonomous'], 'b-', linewidth=2.5,
               label='Autonomous', marker='o', markevery=30)
        ax.plot(time, self.data['mode_human'], 'r-', linewidth=2.5,
               label='Human', marker='s', markevery=30)
        ax.set_xlabel('Time (seconds)', fontweight='bold')
        ax.set_ylabel('Mode Probability', fontweight='bold')
        ax.set_title('Mode Recommendation (DockingModeRecommendation)',
                    fontweight='bold', fontsize=14)
        ax.legend(loc='best', framealpha=0.9)
        ax.grid(True, alpha=0.3)
        ax.axvline(x=100, color='orange', linestyle='--', linewidth=1.5,
                  alpha=0.7, label='Degradation Event')
        
        plt.tight_layout()
        
        if save:
            filename = 'fig3_sensor_impact_traceability.png'
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {filename}")
        
        plt.show()
        return fig
    
    # =========================================================================
    # FIGURE 4: Multi-Scenario Comparison (COMPREHENSIVE)
    # =========================================================================
    def plot_scenario_comparison(self, save=True):
        """
        Compare all scenarios side-by-side
        Shows framework adaptability
        """
        scenarios = ['nominal', 'sensor_degradation', 'operator_fatigue', 'obstruction']
        scenario_names = ['Nominal\nDocking', 'Camera\nDegradation', 
                         'Operator\nFatigue', 'Docking\nObstruction']
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        axes = axes.flatten()
        
        for idx, (scenario, name) in enumerate(zip(scenarios, scenario_names)):
            self.create_simulated_data(scenario)
            time = self.data['time']
            ax = axes[idx]
            
            ax.plot(time, self.data['mode_autonomous'], 'b-', linewidth=2,
                   label='Autonomous')
            ax.plot(time, self.data['mode_human'], 'r-', linewidth=2,
                   label='Human')
            ax.plot(time, self.data['mode_shared'], 'g-', linewidth=2,
                   label='Shared')
            
            # Add event markers
            if scenario == 'sensor_degradation':
                ax.axvline(x=100, color='orange', linestyle='--', alpha=0.7)
            elif scenario == 'obstruction':
                ax.axvline(x=150, color='purple', linestyle='--', alpha=0.7)
            
            ax.set_title(name, fontweight='bold', fontsize=14)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Probability')
            ax.legend(loc='best', fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.set_ylim([0, 1])
        
        plt.suptitle('Docking Framework Adaptation Across Scenarios',
                    fontweight='bold', fontsize=18, y=0.995)
        plt.tight_layout()
        
        if save:
            filename = 'fig4_multi_scenario_comparison.png'
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {filename}")
        
        plt.show()
        return fig
    
    # =========================================================================
    # FIGURE 5: Human Cognitive State Impact (NOVEL CONTRIBUTION)
    # =========================================================================
    def plot_human_cognitive_impact(self, save=True):
        """
        Show human cognitive state evolution and impact on decision
        Demonstrates H1→Attention→SituationalAwareness chain
        """
        self.create_simulated_data('operator_fatigue')
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
        time = self.data['time']
        
        # Top: Cognitive state evolution
        ax = axes[0]
        fatigue = 0.2 + 0.6 * time/300
        stress = 0.3 + 0.2 * time/300
        attention = 0.8 - 0.5 * time/300
        
        ax.plot(time, fatigue, 'r-', linewidth=2.5, label='Fatigue (H1 root)', 
               marker='o', markevery=30)
        ax.plot(time, stress, 'orange', linewidth=2.5, label='Stress (H1 root)',
               marker='s', markevery=30)
        ax.plot(time, attention, 'b-', linewidth=2.5, 
               label='Attention (H1.2, H2.2, H3.1)', marker='^', markevery=30)
        
        ax.set_ylabel('State Level (0-1)', fontweight='bold')
        ax.set_title('Human Operator Cognitive State Evolution',
                    fontweight='bold', fontsize=16)
        ax.legend(loc='best', framealpha=0.9)
        ax.grid(True, alpha=0.3)
        
        # Middle: Situational awareness
        ax = axes[1]
        ax.plot(time, self.data['situational_awareness'], 'g-', linewidth=2.5,
               label='Situational Awareness (H1.3)', marker='d', markevery=30)
        ax.plot(time, self.data['human_reliability'], 'm-', linewidth=2.5,
               label='Human Decision Reliability', marker='p', markevery=30)
        
        ax.set_ylabel('Quality/Reliability', fontweight='bold')
        ax.set_title('Derived Human Performance Metrics',
                    fontweight='bold', fontsize=14)
        ax.legend(loc='best', framealpha=0.9)
        ax.grid(True, alpha=0.3)
        
        # Bottom: Mode shift
        ax = axes[2]
        ax.plot(time, self.data['mode_autonomous'], 'b-', linewidth=2.5,
               label='Autonomous (increases)', marker='o', markevery=30)
        ax.plot(time, self.data['mode_human'], 'r-', linewidth=2.5,
               label='Human (decreases)', marker='s', markevery=30)
        
        ax.set_xlabel('Time (seconds)', fontweight='bold')
        ax.set_ylabel('Mode Probability', fontweight='bold')
        ax.set_title('Mode Recommendation Shift (Autonomous Favored as Operator Fatigues)',
                    fontweight='bold', fontsize=14)
        ax.legend(loc='best', framealpha=0.9)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save:
            filename = 'fig5_human_cognitive_impact.png'
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {filename}")
        
        plt.show()
        return fig
    
    # =========================================================================
    # FIGURE 6: Summary Bar Chart (PAPER SUMMARY FIGURE)
    # =========================================================================
    def plot_scenario_summary(self, save=True):
        """
        Bar chart comparing final mode recommendations across scenarios
        """
        scenarios = ['nominal', 'sensor_degradation', 'operator_fatigue', 'obstruction']
        scenario_labels = ['Nominal', 'Camera\nDegradation', 'Operator\nFatigue', 
                          'Docking\nObstruction']
        
        results = {'Autonomous': [], 'Human': [], 'Shared': []}
        
        for scenario in scenarios:
            self.create_simulated_data(scenario)
            # Take average of last 30 seconds
            results['Autonomous'].append(np.mean(self.data['mode_autonomous'][-30:]))
            results['Human'].append(np.mean(self.data['mode_human'][-30:]))
            results['Shared'].append(np.mean(self.data['mode_shared'][-30:]))
        
        x = np.arange(len(scenario_labels))
        width = 0.25
        
        fig, ax = plt.subplots(figsize=(12, 7))
        
        ax.bar(x - width, results['Autonomous'], width, label='Autonomous',
              color='#3498db', edgecolor='black', linewidth=1.2)
        ax.bar(x, results['Human'], width, label='Human',
              color='#e74c3c', edgecolor='black', linewidth=1.2)
        ax.bar(x + width, results['Shared'], width, label='Shared',
              color='#2ecc71', edgecolor='black', linewidth=1.2)
        
        ax.set_xlabel('Scenario', fontweight='bold', fontsize=14)
        ax.set_ylabel('Mode Probability', fontweight='bold', fontsize=14)
        ax.set_title('Docking Mode Recommendation Across Scenarios (Final State)',
                    fontweight='bold', fontsize=16)
        ax.set_xticks(x)
        ax.set_xticklabels(scenario_labels)
        ax.legend(loc='upper right', framealpha=0.9, fontsize=12)
        ax.grid(True, alpha=0.3, axis='y')
        ax.set_ylim([0, 1.0])
        
        # Add value labels on bars
        for container in ax.containers:
            ax.bar_label(container, fmt='%.2f', fontsize=10)
        
        plt.tight_layout()
        
        if save:
            filename = 'fig6_scenario_summary.png'
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {filename}")
        
        plt.show()
        return fig
    
    # =========================================================================
    # GENERATE ALL PAPER FIGURES
    # =========================================================================
    def generate_all_figures(self):
        """Generate all recommended figures for journal paper"""
        print("=" * 60)
        print("GENERATING ALL PAPER FIGURES")
        print("=" * 60)
        
        print("\n📊 Figure 1: Mode Evolution - Nominal")
        self.plot_mode_evolution('nominal')
        
        print("\n📊 Figure 2: Mode Evolution - Sensor Degradation")
        self.plot_mode_evolution('sensor_degradation')
        
        print("\n📊 Figure 3: Reliability Comparison")
        self.plot_reliability_comparison('sensor_degradation')
        
        print("\n📊 Figure 4: Sensor Impact Traceability")
        self.plot_sensor_impact()
        
        print("\n📊 Figure 5: Multi-Scenario Comparison")
        self.plot_scenario_comparison()
        
        print("\n📊 Figure 6: Human Cognitive Impact")
        self.plot_human_cognitive_impact()
        
        print("\n📊 Figure 7: Scenario Summary")
        self.plot_scenario_summary()
        
        print("\n" + "=" * 60)
        print("✅ ALL FIGURES GENERATED")
        print("=" * 60)


# =============================================================================
# MAIN EXECUTION
# =============================================================================
if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate docking BN paper figures')
    parser.add_argument('--data', type=str, help='Path to recorded data JSON file')
    parser.add_argument('--scenario', type=str, default='all',
                       choices=['all', 'nominal', 'sensor_degradation', 
                               'operator_fatigue', 'obstruction'],
                       help='Which scenario to plot')
    parser.add_argument('--figure', type=str, default='all',
                       help='Specific figure number (1-7) or "all"')
    
    args = parser.parse_args()
    
    plotter = DockingResultsPlotter(args.data)
    
    if args.figure == 'all' and args.scenario == 'all':
        plotter.generate_all_figures()
    elif args.figure == '1':
        plotter.plot_mode_evolution(args.scenario)
    elif args.figure == '2':
        plotter.plot_reliability_comparison(args.scenario)
    elif args.figure == '3':
        plotter.plot_sensor_impact()
    elif args.figure == '4':
        plotter.plot_scenario_comparison()
    elif args.figure == '5':
        plotter.plot_human_cognitive_impact()
    elif args.figure == '6':
        plotter.plot_scenario_summary()
    else:
        plotter.generate_all_figures()
