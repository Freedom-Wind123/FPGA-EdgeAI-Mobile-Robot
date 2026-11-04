import unittest,io,math
from fpga_link import *
from mission_console import Observations

class LinkTests(unittest.TestCase):
 def test_roundtrip(self):
  for kind in Kind:
   b=encode(kind,65535,bytes(range(24)))
   self.assertEqual(decode(b),(kind,65535,bytes(range(24))))
 def test_corruption(self):
  b=encode(Kind.STOP,4)
  for i in range(32):
   c=bytearray(b);c[i]^=1
   with self.assertRaises(ValueError):decode(c)
 def test_no_auto_arm(self):
  w=io.BytesIO();l=TaskLink(w);self.assertEqual(w.getvalue(),b'')
  l.stop();self.assertEqual(decode(w.getvalue())[0],Kind.STOP)
 def test_target(self):
  l=TaskLink(io.BytesIO());b=l.target(.2,800)
  self.assertEqual(struct.unpack('<iiB',decode(b)[2][:9]),(13107,800,1))
  with self.assertRaises(ValueError):l.target(float('nan'),800)
 def test_limits(self):
  l=TaskLink(io.BytesIO())
  for f,args in [(l.manual,(1,0,0)),(l.manual,(0,0,float('inf'))),(l.map_goal,(8,0))]:
   with self.assertRaises(ValueError):f(*args)
 def test_wrap(self):
  l=TaskLink(io.BytesIO(),65535);l.stop();self.assertEqual(l.sequence,0)
 def test_bearing(self):
  self.assertEqual(bearing_from_pixel(320,640,60),0)
  self.assertAlmostEqual(bearing_from_pixel(0,640,60),math.pi/6)
 def test_short_write(self):
  class W:
   def write(self,b):return 2
  with self.assertRaises(IOError):TaskLink(W()).stop()
 def test_observation_freshness(self):
  o=Observations();data={'timestamp_ms':1000,'objects':[{'id':3,'cls':39,'color':1,'confidence':.9,'bearing_rad':.1,'range_mm':900}]}
  self.assertEqual(o.select(data,1050,39,1)['id'],3)
  self.assertIsNone(o.select(data,1060,39,1))
  data['timestamp_ms']=1100;data['objects'][0]['id']=4
  self.assertIsNone(o.select(data,1120,39,1))
  data['objects'][0]['id']=3;data['timestamp_ms']=1200
  self.assertIsNone(o.select(data,1400,39,1))

if __name__=='__main__':unittest.main()
