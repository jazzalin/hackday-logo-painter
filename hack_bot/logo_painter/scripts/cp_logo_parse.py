"""Parse logo coordinates and generate GPS Waypoint Navigation mission file"""
import cv2
import numpy as np
import matplotlib.pyplot as plt

DATUM_N = 4816533.332962739
DATUM_E = 536663.6343794166
REF_X = 4
REF_Y = 10

# Enable dry run mode
DRY = False

class LogoParser:

    def __init__(self, filename):
        self.img = cv2.pyrDown(cv2.imread(filename, cv2.IMREAD_UNCHANGED))
        # self.main_contour
        self.extract_contours()

    def extract_contours(self):
        # threshold image
        ret, threshed_img = cv2.threshold(cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY),
                        127, 255, cv2.THRESH_BINARY)
        # find contours and get the external one
        self.contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE,
                        cv2.CHAIN_APPROX_SIMPLE)
        

    def get_bounding_box(self):
        pass
        # x, y, w, h = cv2.boundingRect(contours[74])
        # print(x,y,w,h)
        # # draw a green rectangle to visualize the bounding rect
        # cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        # cv2.drawContours(img, contours, -1, (255, 255, 0), 1)

    def get_polygon(self):
        # TODO: actually find longest contour using np.mode
        cnt = self.contours[74]
        epsilon = 0.003 * cv2.arcLength(cnt, True)
        # get approx polygons
        poly = cv2.approxPolyDP(cnt, epsilon, True)
        # draw approx polygons
        # cv2.drawContours(self.img, [poly], -1, (0, 255, 0), 1)

        # transform coordinates
        approx = np.squeeze(poly)
        # approx = np.vstack((approx, approx[0])) # repeat first coordinate to complete polygon
        approx[:, 1] = -approx[:, 1]
        approx -= approx[0]
        approx = approx / 50.
        t = np.array([4, 10]) # translate based on datum point in map frame
        approx += t
        return approx
    

    def plot_coordinates(self):
        pass
        # cv2.imshow("contours", img)
        # ESC = 27
        # while True:
        #     keycode = cv2.waitKey(20) & 0xFF
        #     if keycode == ESC:
        #         break
        # cv2.destroyAllWindows()

def generate_mission_file(coord):
    """ Mission plan:
        1) go to first logo coordinate --> task: start painting
        2) draw logo                   --> task: stop painting
        3) return to base              --> task: none
    """
    # Mission 0
    num = coord.shape[0]
    f= open("logo_mission.yaml","w+")
    f.write("# Datum: North: %f, East: %f\n" %(DATUM_N, DATUM_E))
    f.write("- name: Mission 0  # Mission 0\n")
    f.write("  datum: [%f, %f]\n" %(DATUM_E, DATUM_N, ))
    f.write("  goalpoint:\n")
    f.write("    position: [%f, %f]\n" %(coord[-1][0], coord[-1][1]))
    f.write("  goalpoint_theta: 0\n")
    f.write("  viapoints:\n")
    f.write("    []\n")
    f.write("  actions:\n")
    f.write("    - name: start_painting\n")
    f.write("      service: paint_start\n")

    # Mission 1
    f.write("- name: Mission 1  # Mission 1\n")
    f.write("  datum: [%f, %f]\n" %(DATUM_E, DATUM_N, ))
    f.write("  goalpoint:\n")
    f.write("    position: [%f, %f]\n" %(coord[0][0], coord[0][1]))
    f.write("  goalpoint_theta: 0\n")
    f.write("  viapoints:\n")
    for j in range(1, num-1):
        f.write("    - position: [%f, %f] #Point %d\n" %(coord[j][0], coord[j][1], j))
    f.write("  actions:\n")
    f.write("    - name: stop_painting\n")
    f.write("      service: paint_stop\n")

    # Mission 2
    f.write("- name: Mission 2  # Mission 2\n")
    f.write("  datum: [%f, %f]\n" %(DATUM_E, DATUM_N, ))
    f.write("  goalpoint:\n")
    f.write("    position: [%f, %f]\n" %(0, 0)) # return to base
    f.write("  goalpoint_theta: 0\n")
    f.write("  viapoints:\n")
    f.write("    - position: [%f, %f] #Point %d\n" %(coord[-1][0], coord[-1][1], j+1))
    f.write("  actions:\n")
    f.write("    []\n")


    f.close()


if __name__ == "__main__":
    # Use OpenCV to parse contour of logo
    parser = LogoParser("cp_logo.jpg")
    coordinates = parser.get_polygon()
    print(coordinates)

    # Generate OCU mission file
    generate_mission_file(coordinates)
