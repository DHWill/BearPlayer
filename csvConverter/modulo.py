import time

inc = 0
while True:
    inc += 1
    mod = inc % 19
    
    if(mod == 0):
        print("Next")
    
    print("mod: " + str(mod) + " inc: " +  str(inc))

    time.sleep(0.25)
