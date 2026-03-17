import FreeSimpleGUI as fg
import threading

pid1 = [0, 0, 0]
pid2 = [0, 0, 0]
thresh = 0.01
jog = 0
bufferLock = threading.Lock()

sliderRange = 1.5


def get_pid1():
    bufferLock.acquire()
    result = pid1
    bufferLock.release()
    return result

def get_pid2():
    bufferLock.acquire()
    result = pid2
    bufferLock.release()
    return result
def get_thresh():
    bufferLock.acquire()
    result = thresh
    bufferLock.release()
    return result

def get_jog():
    bufferLock.acquire()
    result = jog
    bufferLock.release()
    return result

layout = [
    [fg.Text("PID Controller #1 (centering)")],
    [fg.Slider((-sliderRange, sliderRange), 0, 0.001, key="p1p", enable_events=True),fg.Slider((-sliderRange, sliderRange), 0, 0.001, key="p1i", enable_events=True),fg.Slider((-sliderRange, sliderRange), 0, 0.001, key="p1d", enable_events=True)],
    [fg.Text("PID Controller #2 (balancing)")],
    [fg.Slider((-sliderRange*2, sliderRange*2), 0, 0.001, key="p2p", enable_events=True),fg.Slider((-sliderRange, sliderRange), 0, 0.001, key="p2i", enable_events=True),fg.Slider((-sliderRange, sliderRange), 0, 0.001, key="p2d", enable_events=True)],
    [[fg.Text("threshold")],
    [fg.Slider((0, 0.01), 0.01, 0.0001, key="thresh",enable_events=True)],
    [[fg.Text("offseter")],
    [fg.Slider((-0.3, 0.3), 0, 0.01, key="jog",enable_events=True)]
    ]
    ]]

window = fg.Window('PID Controller Controller', layout)

def main():
    global pid1
    global pid2
    global thresh
    global jog
    while True:
        event, values = window.read()
        
        if event == fg.WIN_CLOSED or event == 'Cancel':
            break
        bufferLock.acquire()
        pid1 = [values["p1p"], values["p1i"], values["p1d"]]
        pid2 = [values["p2p"], values["p2i"], values["p2d"]]
        thresh = values['thresh']
        jog = values["jog"]
        bufferLock.release()
        # print(pid1, pid2)
    window.close()

thread = threading.Thread(target=main)
thread.start()
