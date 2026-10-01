"""Local enrollment and face greeting, independent of chassis motion."""
import os
import time
import cv2
import numpy as np


class FaceGreeting(object):
    def __init__(self, root, voice):
        self.voice = voice
        self.name = ''
        self.score = 0.0
        self.error = ''
        self.ready = False
        self.last_check = -10.0
        self.last_seen = -120.0
        self.last_attempt = -120.0
        self.streak = 0
        self.greeted = False
        try:
            models = os.path.join(root, 'face_models')
            legacy = os.path.join(models, 'face_detection_yunet_2022mar.onnx')
            version = tuple(int(x) for x in cv2.__version__.split('.')[:2])
            detector = legacy if version < (4, 8) else os.path.join(models, 'face_detection_yunet_2023mar.onnx')
            self.detector = cv2.FaceDetectorYN_create(detector, '', (320, 320), 0.75, 0.3, 5000)
            self.recognizer = cv2.FaceRecognizerSF_create(os.path.join(models, 'face_recognition_sface_2021dec.onnx'), '')
            self.templates = []
            folder = os.path.join(root, 'face_db')
            for filename in sorted(os.listdir(folder)):
                if filename.lower().endswith(('.jpg', '.jpeg', '.png')):
                    image = cv2.imread(os.path.join(folder, filename))
                    features = self.features(image) if image is not None else []
                    if features:
                        self.templates.append(features[0])
            if not self.templates:
                raise RuntimeError('No enrollment faces found')
            self.ready = True
        except Exception as error:
            self.error = str(error)

    def features(self, frame):
        h, w = frame.shape[:2]
        scale = min(1.0, 640.0 / max(h, w))
        if scale < 1:
            frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
        self.detector.setInputSize((frame.shape[1], frame.shape[0]))
        _, faces = self.detector.detect(frame)
        result = []
        if faces is not None:
            for face in sorted(faces, key=lambda f: f[2] * f[3], reverse=True):
                if min(face[2], face[3]) < 36:
                    continue
                feature = self.recognizer.feature(self.recognizer.alignCrop(frame, face)).flatten()
                norm = np.linalg.norm(feature)
                if norm > 1e-8:
                    result.append(feature / norm)
        return result

    def update(self, frame):
        now = time.monotonic()
        if not self.ready or now - self.last_check < 1.5:
            return
        self.last_check = now
        try:
            features = self.features(frame)
            self.score = max([float(np.mean(sorted(float(np.dot(f, t)) for t in self.templates)[-2:]))
                              for f in features] or [0.0])
            self.error = ''
            if self.score >= 0.48:
                if now - self.last_seen > 10:
                    self.greeted = False
                self.streak = self.streak + 1 if now - self.last_seen <= 4 else 1
                self.last_seen = now
                self.name = 'Trọng Nguyên' if self.streak >= 2 else ''
                if self.name and not self.greeted and now - self.last_attempt >= 5:
                    self.last_attempt = now
                    self.greeted = self.voice.greet('Xin chào Trọng Nguyên.')
            else:
                self.streak = 0
                self.name = ''
            if self.voice.error:
                self.greeted = False
        except Exception as error:
            self.error = str(error)
