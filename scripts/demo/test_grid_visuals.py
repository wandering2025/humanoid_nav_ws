from types import SimpleNamespace as S
import unittest
import numpy as np
from grid_visuals import cell_data


def grid():
    return S(data=[-1,0,50,100],info=S(width=2,height=2,resolution=.5,
        origin=S(position=S(x=1.,y=2.),orientation=S(x=0.,y=0.,z=0.,w=1.))))

class Cells(unittest.TestCase):
    def test_map_cell_centres_and_unknown_exclusion(self):
        data,n=cell_data(grid());p=np.frombuffer(data,dtype='<f4').reshape(n,4)
        self.assertEqual(n,3)
        np.testing.assert_allclose(p[:,:2],[[1.75,2.25],[1.25,2.75],[1.75,2.75]])

    def test_costmap_does_not_paint_free_or_unknown_cells(self):
        data,n=cell_data(grid(),'local');self.assertEqual(n,2)
        p=np.frombuffer(data,dtype='<f4').reshape(n,4)
        np.testing.assert_allclose(p[:,:2],[[1.25,2.75],[1.75,2.75]])

    def test_source_grid_is_never_modified(self):
        m=grid();original=list(m.data);cell_data(m,'global');self.assertEqual(m.data,original)

if __name__=='__main__':unittest.main()
