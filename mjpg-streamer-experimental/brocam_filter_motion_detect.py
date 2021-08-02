
import cv2
import numpy as np

class MyFilter:
    
    lastFrame = np.array([])

    # kernel for image dilation
    kernel = np.ones((4,4),np.uint8)

    # font style
    font = cv2.FONT_HERSHEY_SIMPLEX

    # directory to save the ouput frames
    pathIn = "/home/pi/debug/"

    image_count = 0
        
    def process(self, img):
        '''
            :param img: A numpy array representing the input image
            :returns: A numpy array to send to the mjpg-streamer output plugin
        '''
        ori_image = img.copy()

        # silly routine that overlays a really large crosshair over the image
        img_h = img.shape[0]
        img_w = img.shape[1]
        
        img_w2 = int(img_w/2)
        img_h2 = int(img_h/2)
        
        # if lastFrame is not empty and img is not empty check for motion
        if(self.lastFrame.size != 0 and img.size != 0):
            # frame differencing
            grayA = cv2.cvtColor(self.lastFrame, cv2.COLOR_BGR2GRAY)
            grayB = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            diff_image = cv2.absdiff(grayB, grayA)
            
            # image thresholding
            ret, thresh = cv2.threshold(diff_image, 30, 255, cv2.THRESH_BINARY)
            
            # image dilation
            dilated = cv2.dilate(thresh,self.kernel,iterations = 1)

            # find contours
            #contours, hierarchy = cv2.findContours(dilated.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
            tst, contours, hierarchy = cv2.findContours(dilated.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

            # shortlist contours appearing in the detection zone
            valid_cntrs = []
            for cntr in contours:
                x,y,w,h = cv2.boundingRect(cntr)
                if (cv2.contourArea(cntr) >= 25):
                    if (cv2.contourArea(cntr) < 40):
                        break
                    valid_cntrs.append(cntr)
                    
            # add contours to original frames
            cv2.drawContours(img, valid_cntrs, -1, (127,200,0), 2)
            
            cv2.putText(img, "motions detected: " + str(len(valid_cntrs)), (55, 15), self.font, 0.6, (0, 180, 0), 2)

            # DEBUG Writing Images to folder /home/pi/debug
#            cv2.imwrite(self.pathIn+str(self.image_count)+'debug.png',grayB)
#            self.image_count = self.image_count+1



           # cv2.line(img, (0, img_h2),(img_w, img_h2),(100, 255, 255))
            #cv2.imwrite(pathIn+str(i)+'.png',img)  

            #font = cv2.FONT_HERSHEY_SIMPLEX
            #cv2.putText(img,'Car Cam',(10,500), font, 4,(255,255,255),2,cv2.LINE_AA)

        self.lastFrame = ori_image

        cv2.line(img, (int(img_w/4), img_h2), (int(3*(img_w/4)), img_h2), (0xff, 0, 0), thickness=3)
        cv2.line(img, (img_w2, int(img_h/4)), (img_w2, int(3*(img_h/4))), (0xff, 0, 0), thickness=3)

        return img
        
def init_filter():
    '''
        This function is called after the filter module is imported. It MUST
        return a callable object (such as a function or bound method). 
    '''
    f = MyFilter()
    return f.process

