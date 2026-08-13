from pynput.keyboard import Key, Controller

keyboard = Controller()
manual_override_blocked = False

def execute_control(command):
    global manual_override_blocked
    
    if command == "FORWARD" and not manual_override_blocked:
        keyboard.press('w')
    elif command == "STOP":
        keyboard.release('w')
        # Trava tentativas manuais enquanto houver risco
        manual_override_blocked = True 

    # Desbloqueia quando a pista estiver livre novamente
    if command == "FORWARD":
        manual_override_blocked = False
