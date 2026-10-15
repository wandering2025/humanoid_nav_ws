import unittest
from gpu_watch import threshold_failure,renderer_verified,ERROR

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
    def test_glx_header_is_not_a_renderer(self):
        self.assertTrue(renderer_verified('Extended renderer info (GLX_MESA_query_renderer):\nOpenGL renderer string: D3D12 (NVIDIA GeForce RTX 4050 Laptop GPU)'))
    def test_intel_and_software_are_rejected(self):
        for value in ('D3D12 (Intel Arc)','llvmpipe','NVIDIA GeForce RTX 4060'):
            self.assertFalse(renderer_verified('OpenGL renderer string: '+value))
    def test_canary_json(self):
        self.assertTrue(renderer_verified('{"renderer":"D3D12 (NVIDIA GeForce RTX 4050 Laptop GPU)"}'))

    def test_graphics_shader_fault_stops_gpu_work(self):
        for text in ('GLSL link result :','active samplers with a different type refer to the same texture image unit','GL_INVALID_OPERATION','D3D12: Removing Device.','corrupted double-linked list','double free or corruption'):
            self.assertIsNotNone(ERROR.search(text))
        self.assertIsNone(ERROR.search('OpenGl version: 4.5 (GLSL 4.5)'))
        self.assertIsNone(ERROR.search('X11 window xid=0x123'))
        self.assertIsNotNone(ERROR.search('NVRM: Xid (PCI:0000:01:00): 31'))

if __name__=='__main__':unittest.main()
