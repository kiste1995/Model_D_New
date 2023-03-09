import rospy
from full_coverage.srv import Cell2pose

def main():
    rospy.init_node("test")

    cli = rospy.ServiceProxy("cell2pose",Cell2pose)
    cell_condition = cli("A1")


    print(cell_condition.x)

    rospy.spin()

if __name__ == "__main__":
    main()