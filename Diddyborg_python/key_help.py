from pynput.keyboard import Key, Listener
def on_press(key):
    print('{0} pressed'.format(
        key))
        
def on_release(key):
    # print('{0} release'.format(
    #     key))
    if key == Key.esc:
        print('Escape key was pressed. Stop Key listener now. Bye!')
        # Stop listener
        return False