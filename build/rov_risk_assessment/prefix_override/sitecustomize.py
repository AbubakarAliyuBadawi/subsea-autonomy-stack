import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/badawi/Desktop/PhD_Autumn_2025/subsea-autonomy-stack/install/rov_risk_assessment'
