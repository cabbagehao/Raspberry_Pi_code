#coding:utf-8
import time
import multiprocessing as mp

from pid import PID 
from car import Car
from encoder import Encoder
from camera_collectFiguresOnGrounds import camera_main2
from infrad import Infrad
from ultrasonic import ul_main


def obstacle_avoidance(car, params, left_v, right_v, v, sample_time):
    if time.time() - params['car_stop_time'] > 2:
        cnt = 10
        left_v *= 0.8
        right_v *= 0.8
        right_v += v*0.3
        for i in range(cnt):
            car.set_speed(right_v, left_v)
            time.sleep(sample_time*3)
        params['car_stop_time'] = time.time()
        params['last_detect_time'] = time.time()
        return
    car.set_speed(0, 0)
    print("Detect obstacle, stop.")
    time.sleep(sample_time)
    return    

def traffic_detect(car, params, share_obj, sample_time)
    pix_value = share_obj.value

    if pix_value > 0:
        params['stopSignals'] += 1
    else:
        params['stopSignals'] = 0
    
    if params['stopSignals'] > 1:
        car.set_speed(0, 0)
        print('red or yellow, stop')
        time.sleep(sample_time)
        if params['stopSignals'] > 600:
            params['stopSignals'] = 0
            pix_value = 0
        return True
    return False

def lane_detect(car, inf, left_v, right_v, v):
    right_v = left_v = 40
    right, left = inf.detect()
    if left == False:
        right_v -= v
        left_v += v
    if right == False:
        right_v += v
        left_v -= v
    car.set_speed(right_v, left_v)  
    return  

def main():
    sample_time = 0.2
    left_v = 30
    pwm_hz = 50
    v = 180
    sample_time = 1.0 / pwm_hz
    
    params = {  'stopSignals': 0,
                'last_detect_time': 0,
                'car_stop_time' : 0
             }
    car = Car()
    inf = Infrad()
    
    # start camera process
    share_obj = mp.Value('i', 0)
    obj_b = mp.Value('i', 0)
    p_camera = mp.Process(target=camera_main2, args=(share_obj, obj_b))
    p_camera.start()
    
    # start ultrasonic process
    share_ul_obj = mp.Value('d', 1000.0)
    p_ul = mp.Process(target=ul_main, args=(share_ul_obj, obj_b))
    p_ul.start()


    while True:
        # Obstacle Detect
        if share_ul_obj.value < 40:
            if time.time() - params['last_detect_time'] < 5:
                pass
            else:
                obstacle_avoidance(car, params, left_v, right_v, v, sample_time)
                continue

        # Traffic Detect
        is_red = traffic_detect(car, params, share_obj)
        if is_red:
            continue

        # Lane Detect
        lane_detect(car, inf, left_v, right_v, v)

        time.sleep(sample_time)


if __name__ == '__main__':
    main()
