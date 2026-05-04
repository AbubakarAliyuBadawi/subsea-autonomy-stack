# 1. Start your docking system
ros2 launch rov_risk_assessment docking_risk_assessment.launch.py

# 2. In another terminal, record data
python3 record_docking_data.py --duration 300 --output nominal_docking.json

# 3. Induce some failure (e.g., cover camera, add fish)
python3 record_docking_data.py --duration 300 --output degraded_docking.json

# 4. Plot real data
python3 plot_docking_results.py --data nominal_docking.json
```

---

## 📋 **My Recommendations for Your Paper:**

### **Must-Have Figures (3-4 figures):**

1. **Figure 2** - Sensor Degradation Mode Evolution
   - Shows framework adapts in real-time
   - Clear mode shift when camera fails

2. **Figure 4** - Sensor Impact Traceability  
   - Proves CoTA → BN mapping claim
   - 3-layer propagation visualization

3. **Figure 6** - Human Cognitive Impact
   - Novel contribution
   - Shows human factors integration

4. **Figure 7** - Scenario Summary
   - Compact overview for results section

### **Optional Figures (if space allows):**

5. **Figure 3** - Reliability Comparison
   - Good for methodology explanation

6. **Figure 5** - Multi-Scenario
   - Comprehensive but takes space

---

## 🎯 **For Your Paper Sections:**

### **Results Section:**
```
"Figure X shows the mode recommendation evolution during sensor 
degradation. At t=100s, camera quality degrades from Good to Poor, 
and ArUco marker visibility drops from All to None (CoTA tasks A2→A3). 
This propagates through VisualGuidanceQuality and DockingReliability 
nodes, causing the framework to shift from Autonomous (75%) to Human 
(42%) control."
```

### **Validation Section:**
```
"Figure Y demonstrates complete traceability from operational tasks 
to probabilistic assessment. Sensor degradation (top panel) propagates 
through intermediate BN nodes (middle panel) to influence the final 
mode decision (bottom panel), validating our systematic mapping from 
CoTA to DBN structure."