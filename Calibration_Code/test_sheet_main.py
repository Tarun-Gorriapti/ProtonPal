import copy
import unittest
import cv2
import numpy as np
from main import DEFAULT_CONFIG, validate_config, tag_sheet_corners, locate_sheet
from vision import Tag, Calibration


class SheetTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        self.k = np.array([[900.,0,640],[0,900.,360],[0,0,1]])
        self.cal = Calibration(self.k,np.zeros(5),1280,720)
        self.r = np.array([.25,-.15,.08])
        self.t = np.array([-.10,-.14,.9])

    def detections(self, ids):
        tags = []
        for s in self.config['tags']:
            if s['id'] not in ids: continue
            image,_ = cv2.projectPoints(tag_sheet_corners(s),self.r,self.t,self.k,self.cal.distortion)
            c = image.reshape(4,2)
            tags.append(Tag(s['id'],c,c.mean(axis=0)))
        return tags

    def test_default_configuration(self):
        validate_config(self.config)
        np.testing.assert_allclose(tag_sheet_corners(self.config['tags'][0])[0], [.01,.01,0])

    def test_center_without_center_tag(self):
        sheet = locate_sheet(self.detections([0,1,2,3]),self.config,self.cal)
        rotation,_ = cv2.Rodrigues(self.r)
        expected = rotation @ np.array([.2159/2,.2794/2,0]) + self.t
        self.assertIsNotNone(sheet)
        np.testing.assert_allclose(sheet['xyz_m'],expected,atol=1e-6)
        self.assertNotIn(4,sheet['used_ids'])

    def test_perspective_pixel_center(self):
        sheet = locate_sheet(self.detections([0,1,2,3,4]),self.config)
        point = np.array([[.2159/2,.2794/2,0]])
        expected,_ = cv2.projectPoints(point,self.r,self.t,self.k,self.cal.distortion)
        np.testing.assert_allclose(sheet['center_px'],expected.reshape(2),atol=.01)
        self.assertIsNone(sheet['xyz_m'])

    def test_missing_and_unknown_targets(self):
        self.assertIsNone(locate_sheet([],self.config,self.cal))
        self.assertIsNone(locate_sheet(self.detections([4]),self.config,self.cal))
        self.assertIsNotNone(locate_sheet(self.detections([0,2]),self.config,self.cal))

    def test_duplicate_and_invalid_config(self):
        tags = self.detections([0,1])
        self.assertIsNone(locate_sheet(tags+[tags[0]],self.config,self.cal))
        for mutation in ('duplicate','outside','overlap'):
            bad = copy.deepcopy(self.config)
            if mutation == 'duplicate': bad['tags'][1]['id'] = 0
            if mutation == 'outside': bad['tags'][0]['x_mm'] = 0
            if mutation == 'overlap':
                bad['tags'][1]['x_mm'] = 25
                bad['tags'][1]['y_mm'] = 25
            with self.assertRaises(ValueError): validate_config(bad)

    def test_rotated_tag(self):
        self.config['tags'][0]['rotation_deg'] = 90
        sheet = locate_sheet(self.detections([0,1,2,3]),self.config,self.cal)
        self.assertIsNotNone(sheet)


if __name__ == '__main__': unittest.main()
