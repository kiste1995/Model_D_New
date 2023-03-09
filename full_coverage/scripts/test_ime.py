import rospy
import time

def main() :
    start = time.time()
    for i in range(10):
        rospy.sleep(0.1)
    time_term = time.time() - start
    print(time_term) 

if __name__ == "__main__":
    main()