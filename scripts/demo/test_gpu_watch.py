import unittest
from gpu_watch import threshold_failure

class Limits(unittest.TestCase):
    def row(self,**kwargs):return dict(temperature_c=47,memory_mib=600,memory_total_mib=6141,utilization_pct=20,**kwargs)
    def test_healthy(self):self.assertIsNone(threshold_failure(self.row(),600,None,0)[0])
    def test_temperature(self):
        r=self.row();r['temperature_c']=75;self.assertIsNotNone(threshold_failure(r,600,None,0)[0])
    def test_memory_growth(self):
        r=self.row();r['memory_mib']=1400;self.assertIsNotNone(threshold_failure(r,600,None,0)[0])
    def test_sustained_load(self):
        r=self.row();r['utilization_pct']=96
        failure,since=threshold_failure(r,600,None,10);self.assertIsNone(failure)
        self.assertIsNotNone(threshold_failure(r,600,since,15)[0])
if __name__=='__main__':unittest.main()
