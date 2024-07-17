import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import hsv_to_rgb
import io
my_dpi=120
colours=c = {a + 1: hsv_to_rgb(np.array([a / float(10), 1, 1])) for a in range(10)}
def get_img_from_fig(fig, dpi=my_dpi):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi)
    buf.seek(0)
    img_arr = np.frombuffer(buf.getvalue(), dtype=np.uint8)
    buf.close()
    img = cv.imdecode(img_arr, 1)
    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)/255
    return img
def update(image,svo,ngh,distri,self):
    section1=np.zeros((360,640,3))
    section2=np.zeros((360,640,3))
    section3=np.ones((360,640,3))*colours[self+1]
    section4=np.ones((360,640,3))*colours[ngh+1]
    x=plt.figure(figsize=(640/my_dpi, 360/my_dpi), dpi=my_dpi)
    print(x)
    ax=x.add_subplot(111)
    print(ax)
    ax.set_ylim(0,0.5)
    ax.set_xlim(0,50)
    ax.set_title("SVO Distribution")
    ax.set_xlabel("Angle (Degrees)")
    ax.set_ylabel("Probability")
    ax.bar(range(0,50,5),distri,width=10,color='blue')
    ax.axvline(x=svo,color='r')
    section1=get_img_from_fig(x)
    plt.close()
    font = cv.FONT_HERSHEY_SIMPLEX
    org = (360//2-150, 640//2-80)
    fontScale = 10
    color = (0, 0, 1)
    thickness = 5
    section2 = cv.putText(section2, "SVO", (0,50), font,
                    2, (1,1,1), thickness, cv.LINE_AA)
    section2 = cv.putText(section2, "{}".format(svo), org, font,
                    8, color, thickness, cv.LINE_AA)
    section3 = cv.putText(section3, "SELF", (0,50), font,
                2, (0,0,0), thickness, cv.LINE_AA)
    section4 = cv.putText(section4, "NEIGHBOR", (0,50), font,
                    2, (0,0,0), thickness, cv.LINE_AA)
    image[0:360,0:600]=section1
    image[360:,0:640]=section2
    image[0:360,640:]=section3
    image[360:,640:]=section4
    return image
image=np.zeros((720,1280,3))
cv.namedWindow("foo", cv.WINDOW_NORMAL)
cv.setWindowProperty("foo", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
# while True:
distri=np.random.random((10))
distri=distri/np.sum(distri)
svo=np.random.random(1)[0]*50
self=np.random.randint(0,10)
ngh=np.random.randint(0,10)
image=update(image,svo,ngh,distri,self)
cv.imshow("foo", image)
cv.waitKey(0)

# for i in
# cv.imshow("foo", image+255)
# cv.waitKey(0)