import importlib.util
from pathlib import Path
import sys
import unittest
spec=importlib.util.spec_from_file_location('person_watch',Path(__file__).resolve().parents[1]/'deploy/person_watch/person_watch.py')
pw=importlib.util.module_from_spec(spec);sys.modules[spec.name]=pw;spec.loader.exec_module(pw)
class PolicyTests(unittest.TestCase):
    def test_confirm_and_cooldown(self):
        p=pw.Confirmation()
        self.assertFalse(p.update(0,True));self.assertTrue(p.update(2,True))
        for t in range(4,32,2):self.assertFalse(p.update(t,True))
        self.assertTrue(p.update(32,True))
    def test_stale_negative_gap_break_confirmation(self):
        for positive,fresh in [(False,True),(True,False)]:
            p=pw.Confirmation();p.update(0,True)
            self.assertFalse(p.update(2,positive,fresh=fresh))
            self.assertFalse(p.update(4,True));self.assertTrue(p.update(6,True))
        p=pw.Confirmation();p.update(0,True)
        self.assertFalse(p.update(10,True));self.assertTrue(p.update(12,True))
    def test_cameras_independent_and_disconnect_preserves_cooldown(self):
        a,b=pw.Confirmation(),pw.Confirmation()
        a.update(0,True);b.update(0,True)
        self.assertTrue(a.update(2,True));self.assertTrue(b.update(2,True))
        a.update(3,False,fresh=False);a.update(4,True);self.assertFalse(a.update(6,True))
    def test_boundaries(self):
        config={'ffmpeg':'/private/ffmpeg','cameras':[{'id':c,'label':c,'url':'rtsp://127.0.0.1:8554/'+c} for c in ('cam3','cam4','cam5')]}
        self.assertEqual(pw.validate_config(config),config)
        for url in ('rtsp://example.com/cam3','rtsp://user:pass@127.0.0.1/cam3','rtsp://127.0.0.1/cam3_normal'):
            changed={**config,'cameras':[dict(c) for c in config['cameras']]};changed['cameras'][0]['url']=url
            with self.assertRaises(ValueError):pw.validate_config(changed)
        with self.assertRaises(ValueError):pw.validate_config({**config,'fps_per_camera':15})
        cmd=pw.decoder_command(config,config['cameras'][0])
        self.assertEqual(cmd[cmd.index('-threads')+1],'1');self.assertLess(cmd.index('-allowed_media_types'),cmd.index('-i'))
    def test_bounded_history(self):
        state=pw.State({'cameras':[{'id':'cam3','label':'Frontal'}]})
        for i in range(100):state.alerts.append({'id':i})
        self.assertEqual(len(state.snapshot()['alerts']),50)
        self.assertEqual(state.snapshot()['cameras'][0]['status'],'starting')
class ConversionTests(unittest.TestCase):
    def test_focus_fold_preserves_convolution_at_edges_and_interior(self):
        import numpy as np
        spec=importlib.util.spec_from_file_location('convert_model',Path(__file__).resolve().parents[1]/'deploy/person_watch/convert_model.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        rng=np.random.default_rng(100)
        image=rng.standard_normal((3,8,8)).astype(np.float32)
        weights=rng.standard_normal((16,12,3,3)).astype(np.float32)
        focus=np.concatenate([image[:,dy::2,dx::2] for dy,dx in ((0,0),(1,0),(0,1),(1,1))])
        focused=np.pad(focus,((0,0),(1,1),(1,1)))
        padded=np.pad(image,((0,0),(2,2),(2,2)))
        folded=module.fold_focus_weights(weights)
        for y in range(4):
            for x in range(4):
                expected=np.einsum('oijk,ijk->o',weights,focused[:,y:y+3,x:x+3])
                actual=np.einsum('oijk,ijk->o',folded,padded[:,y*2:y*2+6,x*2:x*2+6])
                np.testing.assert_allclose(actual,expected,atol=1e-5,rtol=1e-5)
        with self.assertRaises(ValueError):module.fold_focus_weights(np.zeros((2,2,2,2)))

if __name__=='__main__':unittest.main()
