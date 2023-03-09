import time

module_controll_command = 0b0000000000000000

def module_controller(*args):
    global module_controll_command

    # |     RESERVE   |       LED        |    Pulifier      |  pump     |Reserve| UVC |
    # | 0  1  2  3  4 |  5     6    7    |  8   9   10  11  | 12   13   |  14   | 15  |
    # | X  X  X  X  X |  LEDG  LEDB LEDR |  L4  L3  L2  L1  | SOL  PUMP |  X    | UVC |

    module_controll_dict = {
        "pump" : 0b0000000000001100,
        "UVC"  : 0b0000000000000001,
        "LEDG" : 0b0000010100000000,
        "LEDB" : 0b0000001100000000,
        "LEDR" : 0b0000100100000000,
        "all"  : 0b1111111111111111
    }

    

    command_list = []
    for i in args :
        if "_off" in i :
            print(i[:-1*(len("_off"))])
            command = ~ module_controll_dict[i[:-1*(len("_off"))]]
            module_controll_command = command & module_controll_command
        elif "_on" in i :
            print(i[:-1*(len("_on"))])
            command = module_controll_dict[i[:-1*(len("_on"))]]
            module_controll_command = command | module_controll_command
        

    module_controll_command_str = str(hex(module_controll_command)).upper()[2:].zfill(4)
    

    print(module_controll_command_str)

    start = time.time()
    print(time.time())
    while ( time.time()- start) != 1:
        pass
    print(time.time())

module_controller("pump_on")
module_controller("UVC_on")
module_controller("LEDG_on","LEDR_on","UVC_off")
module_controller("pump_off","LEDB_on")
module_controller("all_off")
module_controller("all_on")


