import pytest
from tbd_podi_behavior.gaze_controller import PodiGazeActuator
import numpy as np
import rospy
import random


SCREEN_WIDTH = 0.3
SCREEN_HEIGHT = 0.15

def test_face_build():
    # test the build face method
    (code, result) = PodiGazeActuator._calculate_screen_location(np.array([5,0,0]), SCREEN_WIDTH, SCREEN_HEIGHT)
    assert code == 0

def test_face_straight_ahead():
    for i in range(0,10):
        # test the build face method
        (code, result) = PodiGazeActuator._calculate_screen_location(np.array([i,0,0]), SCREEN_WIDTH, SCREEN_HEIGHT)
        assert code == 0
        assert result[0] == pytest.approx(0.5)
        assert result[1] == pytest.approx(0.5)

def test_behind_head():
    for i in range(0,100):
        # test a bunch of numbers behind the head, just in case
        # test the build face method
        rand_x = random.uniform(-100,100)
        rand_y = random.uniform(-100,100)
        rand_z = random.uniform(-0.01,-100)
        (code, result) = PodiGazeActuator._calculate_screen_location(np.array([rand_z,rand_x,rand_y]), SCREEN_WIDTH, SCREEN_HEIGHT)
        assert code == 2  

def test_directly_above_head():
    (code, result) = PodiGazeActuator._calculate_screen_location(np.array([0,0,10]), SCREEN_WIDTH, SCREEN_HEIGHT)
    assert code == 3  
    (code, result) = PodiGazeActuator._calculate_screen_location(np.array([0,0,-10]), SCREEN_WIDTH, SCREEN_HEIGHT)
    assert code == 3 

def test_face_towards_left(capsys):
    # test the build face method
    previous = -1
    for i in range(1,10):
        (code, result) = PodiGazeActuator._calculate_screen_location(np.array([5,i,0]), SCREEN_WIDTH, SCREEN_HEIGHT)
        assert code == 0
        if previous != -1:
            assert previous > result[0]
        assert result[1] == pytest.approx(0.5)
        previous = result[0]

def test_face_towards_right(capsys):
    # test the build face method
    previous = -1
    for i in range(0,10):
        (code, result) = PodiGazeActuator._calculate_screen_location(np.array([5,-1* i,0]), SCREEN_WIDTH, SCREEN_HEIGHT)
        assert code == 0
        if previous != -1:
            assert previous < result[0]
        assert result[1] == pytest.approx(0.5)
        previous = result[0]

def test_face_towards_up(capsys):
    # test the build face method
    previous = None
    for i in range(0,5):
        (code, result) = PodiGazeActuator._calculate_screen_location(np.array([5,0,i]), SCREEN_WIDTH, SCREEN_HEIGHT)
        assert code == 0
        if previous is not None:
            assert previous > result[1]
        assert result[0] == pytest.approx(0.5)
        previous = result[1]

def test_face_towards_down(capsys):
    # test the build face method
    previous = -1
    for i in range(0,5):
        (code, result) = PodiGazeActuator._calculate_screen_location(np.array([5,0,-1 * i]), SCREEN_WIDTH, SCREEN_HEIGHT)
        assert code == 0
        if previous is not None:
            assert previous < result[1]
        assert result[0] == pytest.approx(0.5)
        previous = result[1]

def test_face_corners():
    (code, result) = PodiGazeActuator._calculate_screen_location(np.array([5,1,1]), SCREEN_WIDTH, SCREEN_HEIGHT)
    assert code == 0
    assert result[0] < 0.5
    assert result[1] < 0.5
    (code, result) = PodiGazeActuator._calculate_screen_location(np.array([5,-1,1]), SCREEN_WIDTH, SCREEN_HEIGHT)
    assert code == 0
    assert result[0] > 0.5
    assert result[1] < 0.5
    (code, result) = PodiGazeActuator._calculate_screen_location(np.array([5,1,-1]), SCREEN_WIDTH, SCREEN_HEIGHT)
    assert code == 0
    assert result[0] < 0.5
    assert result[1] > 0.5
    (code, result) = PodiGazeActuator._calculate_screen_location(np.array([5,-1,-1]), SCREEN_WIDTH, SCREEN_HEIGHT)
    assert code == 0
    assert result[0] > 0.5
    assert result[1] > 0.5