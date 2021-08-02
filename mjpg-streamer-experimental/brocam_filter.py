
import sys
import cv2
import numpy as np
import datetime
import threading
import pantilthat
from collections import Mapping, Container

class MyFilter:
    

    # kernel for image dilation
    kernel = np.ones((4,4),np.uint8)

    # font style
    font = cv2.FONT_HERSHEY_SIMPLEX

    # directory to save the ouput frames
    pathIn = "/home/pi/debug/"

    debugMode = True
    debugModeWriteImage = True

    frame_count = 0
    consecutive_frame = 3       # this provides better resolution of a moving object, but the higher the value the more of the moving objects "ghost" it will catch, a larger area covering where it once
    background_image_count = 0
    setup_initialised = False   # determine if ready to start processing or build up initialisation data
    background_initialised = False # determine if background is available (finsihed in thread)
    background_frames = []      # store for background frames - will use background_frame_count_span for length
    frame_diff_list = []
    background_median_size = 10         # 25
    background_frame_count_span = 150   # Number of frames to save for determining the background (assume approx 5 fps) - should always be higher than background_median_size 150
    background = np.array([])
    o_datetime = 0
    last_background_update_ts = 0
    background_update_time = 300    # in seconds, how often to regenerate the background
    rebuild_background = False
    is_background_building = False
    pthPan = 0
    pthTilt = 0

    def process(self, img):
        '''
            :param img: A numpy array representing the input image
            :returns: A numpy array to send to the mjpg-streamer output plugin
        '''
        # Constants
        motionThreshold = 500
        self.pthPan = pantilthat.get_pan ()
        self.pthTilt = pantilthat.get_tilt()
        #print("Pan: "+str(self.pthPan))
        #print("Tilt: "+str(self.pthTilt))
#        ori_image = img.copy()

        # get screen dimensions
        img_h = img.shape[0]
        img_w = img.shape[1]
        
        img_w2 = int(img_w/2)
        img_h2 = int(img_h/2)

        valid_cntrs = 0

       # self.background = img.copy() # just set up the variable with cur image

        # get datetime object
        self.o_datetime = datetime.datetime.now()


        # if not initialised, setup the background image - take x amount of frames and use a random sample of y frames to determine a median image which will remove currently moving objects
        if(self.setup_initialised == False):
            # add image to background_frames list
            self.background_frames.append(img.copy())
            self.background_image_count = self.background_image_count + 1

            if(self.debugMode):
                print("Debug Initialising Frame: "+str(self.background_image_count))

            # if threshold reached do the median calculation and finish initialisation by setting to true
            if(self.background_image_count >= self.background_frame_count_span):
                # self.process_background(self.background_median_size)
                self.setup_initialised = True
                self.is_background_building = True
                th = threading.Thread(target=self.process_background)
                th.start()
                
        else:
            if(self.background_initialised):
                self.frame_count = self.frame_count + 1
                
                # remove first frame from background_frames and then add img
                #self.background_frames.pop(0)  # pop causes memoryleak
                del self.background_frames[0]
                self.background_frames.append(img.copy())
                
                #if(self.debugMode):
                #    print("Number of background_frames: "+str(len(self.background_frames)))
                
                # IMPORTANT STEP: convert the frame to grayscale first
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
         
                # find the difference between current frame and base frame
                frame_diff = cv2.absdiff(gray, self.background)
                # thresholding to convert the frame to binary
#                ret, thres = cv2.threshold(frame_diff, 50, 255, cv2.THRESH_BINARY)
                ret, thres = cv2.threshold(frame_diff, 40, 255, cv2.THRESH_BINARY)
                # dilate the frame a bit to get some more white area...
                # ... makes the detection of contours a bit easier
                dilate_frame = cv2.dilate(thres, None, iterations=1)
                # append the final result into the `frame_diff_list`
                #if(len(self.frame_diff_list) > 0 and len(self.frame_diff_list) <= self.consecutive_frame):
                if(len(self.frame_diff_list) >= self.consecutive_frame):
                    del self.frame_diff_list[0]
                    #self.frame_diff_list.pop(0)    # pop causes memoryleak

                self.frame_diff_list.append(dilate_frame)

                #if(self.debugMode):
                #    print("Number of frame_diff_list: "+str(len(self.frame_diff_list)))

                # if we have reached `consecutive_frame` number of frames
               #if len(self.frame_diff_list) == self.consecutive_frame:
                # add all the frames in the `frame_diff_list`
                sum_frames = sum(self.frame_diff_list)
                # find the contours around the white segmented areas
                #ret, contours, hierarchy = cv2.findContours(sum_frames, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                ret, contours, hierarchy = cv2.findContours(sum_frames, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                # draw the contours, not strictly necessary
               # if(self.debugMode):
                   # for i, cnt in enumerate(contours):
                    #    cv2.drawContours(img, contours, i, (0, 0, 255), 3)


                for contour in contours:
                    # continue through the loop if contour area is less than 500...
                    # ... helps in removing noise detection
                    if cv2.contourArea(contour) < motionThreshold:
                        continue
                    # get the xmin, ymin, width, and height coordinates from the contours
                    (x, y, w, h) = cv2.boundingRect(contour)
                    # draw the bounding boxes
                    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    valid_cntrs = valid_cntrs + 1
        #            cv2.imshow('Detected Objects', orig_frame)
                




            # motions detected
#            valid_cntrs = []

            # if lastFrame is not empty and img is not empty check for motion
#            if(self.lastFrame.size != 0 and img.size != 0 and self.setup_initialised):
                # frame differencing
#                grayA = cv2.cvtColor(self.lastFrame, cv2.COLOR_BGR2GRAY)
#                grayB = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#                diff_image = cv2.absdiff(grayB, grayA)
                
                # image thresholding
#                ret, thresh = cv2.threshold(diff_image, 30, 255, cv2.THRESH_BINARY)
                
                # image dilation
#                dilated = cv2.dilate(thresh,self.kernel,iterations = 1)

                # find contours
                #contours, hierarchy = cv2.findContours(dilated.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
#                tst, contours, hierarchy = cv2.findContours(dilated.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

                # shortlist contours appearing in the detection zone
                
#                for cntr in contours:
#                    x,y,w,h = cv2.boundingRect(cntr)
#                    if (cv2.contourArea(cntr) >= motionThreshold):
                        #if (cv2.contourArea(cntr) < 40):
                        #    break
#                        valid_cntrs.append(cntr)
                        
                # add contours to original frames
#                cv2.drawContours(img, valid_cntrs, -1, (127,200,0), 2)
                
                # DEBUG Writing Images to folder /home/pi/debug
#                if(self.debugMode):
#                    cv2.imwrite(self.pathIn+str(self.frame_count)+'debug.png',grayB)


#        self.lastFrame = ori_image

        ### Image processing to img - do at end to preserve image for next frame

        # Show detected number of motions
        if(self.setup_initialised and self.background_initialised):
            cv2.putText(img, "motions detected: " + str(valid_cntrs), (55, 15), self.font, 0.6, (0, 180, 0), 2)

        if(self.is_background_building):
            cv2.putText(img, "Building background", (55, 36), self.font, 0.6, (0, 180, 0), 2)

        # crude crosshair
        #cv2.line(img, (int(img_w/4), img_h2), (int(3*(img_w/4)), img_h2), (0xff, 0, 0), thickness=3)
        #cv2.line(img, (img_w2, int(img_h/4)), (img_w2, int(3*(img_h/4))), (0xff, 0, 0), thickness=3)
        if(self.debugMode):
            crosshair_size = 12
            cv2.line(img, (int(img_w2 - (img_w/crosshair_size)), img_h2), (int(img_w2 + (img_w/crosshair_size)), img_h2), (0xff, 0, 0), thickness=3)
            cv2.line(img, (img_w2, (int(img_h2 - (img_h/crosshair_size)))), (img_w2, int(img_h2 + (img_h/crosshair_size))), (0xff, 0, 0), thickness=3)

        # Info box
        margin = 20
        numTextLines = 7 # doesn't include Title
        fontscale = 0.6

        # Info box - text strings
        titleText = "BroNet Car Cam"
        dateText = self.o_datetime.strftime("%d-%m-%Y")
        timeText = self.o_datetime.strftime("%X")
        panText = "Cam Pan Angle: "+str(self.pthPan)
        tiltText = "Cam Tilt Angle: "+str(self.pthTilt) 

        # Info box - calculate longest string to determine positioning
        textsize_titletext = cv2.getTextSize(titleText, self.font, 1, 2)[0]
        textsize_dateText = cv2.getTextSize(dateText, self.font, fontscale, 1)[0]
        textsize_timeText = cv2.getTextSize(timeText, self.font, fontscale, 1)[0]
        textsize_pantext = cv2.getTextSize(panText, self.font, fontscale, 1)[0]
        textsize_tilttext = cv2.getTextSize(tiltText, self.font, fontscale, 1)[0]

        # get coords based on boundary
        textX = textsize_dateText[0]
        textY = textsize_dateText[1]+6

        # see if titleText is wider only
        if(textsize_titletext[0] > textX):
            textX = textsize_titletext[0]

        # see if timeText is bigger
        if(textsize_timeText[0] > textX):
            textX = textsize_timeText[0]

        if(textsize_timeText[1] > textY):
            textY = textsize_timeText[1]

        # see if panText is bigger
        if(textsize_pantext[0] > textX):
            textX = textsize_pantext[0]

        if(textsize_pantext[1] > textY):
            textY = textsize_pantext[1]
        # see if tiltText is bigger
        if(textsize_tilttext[0] > textX):
            textX = textsize_tilttext[0]

        if(textsize_tilttext[1] > textY):
            textY = textsize_tilttext[1]

        posTextX = img_w - textX - margin
        posTextY = img_h - (textY * numTextLines) - margin     # set size of box based on biggest textY * number of text lines

        # Info box - write text to image
        cv2.putText(img, titleText, (posTextX, posTextY - textsize_titletext[1]), self.font, 1, (93, 93, 216), 2)
        cv2.putText(img, dateText, (posTextX, posTextY), self.font, fontscale, (0, 180, 0), 1)
        cv2.putText(img, timeText, (posTextX, posTextY + textY), self.font, fontscale, (0, 180, 0), 1)
        cv2.putText(img, panText, (posTextX, posTextY + (textY*2)), self.font, fontscale, (0, 180, 0), 1)
        cv2.putText(img, tiltText, (posTextX, posTextY + (textY*3)), self.font, fontscale, (0, 180, 0), 1)
        # end Info box

        # Debug box
        if(self.debugMode):
            debugTitleText = "Debug Information"
            debugFramesText = "Background Frames #: "
            debugFrameDiffText = "Difference Frames #: "
            debugSampleSize = "Background sample size: "
            debugLastUpdText = "Background last update: "
            debugUpdateTime = "Background updates every: "
            debugNextUpdate = "Next Background update in: "
            
            # Info box - calculate longest string to determine positioning
            textsize_debugTitleText = cv2.getTextSize(debugTitleText, self.font, 1, 2)[0]
            textsize_debugFramesText = cv2.getTextSize(debugFramesText, self.font, fontscale, 1)[0]
            textsize_debugFrameDiffText = cv2.getTextSize(debugFrameDiffText, self.font, fontscale, 1)[0]
            textsize_debugSampleSize = cv2.getTextSize(debugSampleSize, self.font, fontscale, 1)[0]
            textsize_debugLastUpdText = cv2.getTextSize(debugLastUpdText, self.font, fontscale, 1)[0]
            textsize_debugUpdateTime = cv2.getTextSize(debugUpdateTime, self.font, fontscale, 1)[0]
            textsize_debugNextUpdate = cv2.getTextSize(debugNextUpdate, self.font, fontscale, 1)[0]

            # add values now length is determined
            #debugFramesText = debugFramesText+str(len(self.background_frames))+" ("+str("{:.2f}".format(sys.getsizeof(self.background_frames)/1024))+" Mb)"
            debugFramesText = debugFramesText+str(len(self.background_frames))+" ("+str("{:.2f}".format(deep_getsizeof(self.background_frames, set())/1024/1024))+" Mb)"
            debugFrameDiffText = debugFrameDiffText+str(len(self.frame_diff_list))+" ("+str("{:.2f}".format(deep_getsizeof(self.frame_diff_list, set())/1024/1024))+" Mb)"
            debugSampleSize = debugSampleSize+str(self.background_median_size)+" frames"
            debugLastUpdText = debugLastUpdText+str(datetime.datetime.fromtimestamp(self.last_background_update_ts).strftime("%X"))
            debugUpdateTime = debugUpdateTime+str(self.background_update_time)+" seconds"
            dbgCountdown = 0
            if(self.last_background_update_ts != 0):
                dbgCountdown = 300 - int(datetime.datetime.now().timestamp()-self.last_background_update_ts)
            debugNextUpdate = debugNextUpdate+str(dbgCountdown)+" seconds"

            # get coords based on boundary
            dbgTextX = textsize_debugFramesText[0]
            dbgTextY = textsize_debugFramesText[1]+6

            # find longest
            if(textsize_debugFrameDiffText[0] > dbgTextX):
                dbgTextX = textsize_debugFrameDiffText[0]

            if(textsize_debugSampleSize[0] > dbgTextX):
                dbgTextX = textsize_debugSampleSize[0]

            if(textsize_debugLastUpdText[0] > dbgTextX):
                dbgTextX = textsize_debugLastUpdText[0]

            if(textsize_debugUpdateTime[0] > dbgTextX):
                dbgTextX = textsize_debugUpdateTime[0]

            if(textsize_debugNextUpdate[0] > dbgTextX):
                dbgTextX = textsize_debugNextUpdate[0]

            dbgPosTextX = margin + dbgTextX
            dbgPosTextY = img_h - (textY * numTextLines) - margin     # set size of box based on biggest textY * number of text lines

            cv2.putText(img, debugTitleText, (margin, dbgPosTextY - textsize_debugTitleText[1]), self.font, 1, (93, 93, 216), 2)
            cv2.putText(img, debugFramesText, (dbgPosTextX-textsize_debugFramesText[0], dbgPosTextY), self.font, fontscale, (0, 180, 0), 1)
            cv2.putText(img, debugSampleSize, (dbgPosTextX-textsize_debugSampleSize[0], dbgPosTextY + (dbgTextY*1)), self.font, fontscale, (0, 180, 0), 1)
            cv2.putText(img, debugFrameDiffText, (dbgPosTextX-textsize_debugFrameDiffText[0], dbgPosTextY + (dbgTextY*2)), self.font, fontscale, (0, 180, 0), 1)
            cv2.putText(img, debugLastUpdText, (dbgPosTextX-textsize_debugLastUpdText[0], dbgPosTextY + (dbgTextY*3)), self.font, fontscale, (0, 180, 0), 1)
            cv2.putText(img, debugUpdateTime, (dbgPosTextX-textsize_debugUpdateTime[0], dbgPosTextY + (dbgTextY*4)), self.font, fontscale, (0, 180, 0), 1)
            cv2.putText(img, debugNextUpdate, (dbgPosTextX-textsize_debugNextUpdate[0], dbgPosTextY + (dbgTextY*5)), self.font, fontscale, (0, 180, 0), 1)
            # end Debug box

        # warn that system is currently initialising so not to expect anything
        if(self.setup_initialised == False or self.rebuild_background == True):
            initMsg = "Initialising Background for Object Motion Detection"
            textsize_initMsg = cv2.getTextSize(initMsg, self.font, 1, 2)[0]
            initMsgX = textsize_initMsg[0]
            initMsgY = textsize_initMsg[1]

            # center text
            centerTextX = img_w2 - int(initMsgX / 2)
            centerTextY = img_h2 - int(initMsgY / 2)
            cv2.putText(img, initMsg, (centerTextX, centerTextY), self.font, 1, (200, 200, 200), 2)

        if(self.rebuild_background):
            #self.process_background(self.background_median_size)
            self.is_background_building = True
            th = threading.Thread(target=self.process_background)
            th.start()
            self.rebuild_background = False
            self.last_background_update_ts = datetime.datetime.now().timestamp() # reset timer else it'll recall itself thinking its due before the function completed

        # update background if time is met
        if(self.last_background_update_ts != 0 and (self.o_datetime.now().timestamp() - self.last_background_update_ts) > self.background_update_time and self.setup_initialised == True):
            self.rebuild_background = True

            initMsg = "Rebuilding Background for Object Motion Detection"
            textsize_initMsg = cv2.getTextSize(initMsg, self.font, 1, 2)[0]
            initMsgX = textsize_initMsg[0]
            initMsgY = textsize_initMsg[1]

            # center text
            centerTextX = img_w2 - int(initMsgX / 2)
            centerTextY = img_h2 - int(initMsgY / 2)
            cv2.putText(img, initMsg, (centerTextX, centerTextY), self.font, 1, (200, 200, 200), 2)

        return img
    

    #def process_background(self, num_sample_frames):
    def process_background(self):
        num_sample_frames = self.background_median_size

        ts1 = datetime.datetime.now().timestamp()
        # we will randomly select 50 frames for the calculating the median
        frame_indices = self.background_frame_count_span * np.random.uniform(size=num_sample_frames)

        # print(frame_indices)
        # store the frames in array
        # print("Calculating background frame using random sample of "+str(num_sample_frames)+" frames from "+str(self.background_frame_count_span)+" images")
        print("Calculating background frame using random sample of "+str(num_sample_frames)+" frames from "+str(len(self.background_frames))+" images")

        frames = []
        for idx in frame_indices:
            # set the frame id to read that particular frame
            frames.append(self.background_frames[idx.astype(np.int64)])

        # calculate the median
        self.background = np.median(frames, axis=0).astype(np.uint8)

        # convert the background model to grayscale format
        self.background = cv2.cvtColor( self.background, cv2.COLOR_BGR2GRAY)
        if(self.debugModeWriteImage):
            cv2.imwrite(self.pathIn+'background_median_debug_'+str(ts1)+'.png', self.background) # debug save
        
        ts2 = datetime.datetime.now().timestamp()
        print("Initialisation of background complete")
        print("Time Taken: "+str(ts2 - ts1))

        self.last_background_update_ts = ts2
        
        self.rebuild_background = False
        self.background_initialised = True
        self.is_background_building = False

        return
        
def init_filter():
    '''
        This function is called after the filter module is imported. It MUST
        return a callable object (such as a function or bound method). 
    '''
    f = MyFilter()
    return f.process

def deep_getsizeof(o, ids):
    """Find the memory footprint of a Python object
 
    This is a recursive function that drills down a Python object graph
    like a dictionary holding nested dictionaries with lists of lists
    and tuples and sets.
 
    The sys.getsizeof function does a shallow size of only. It counts each
    object inside a container as pointer only regardless of how big it
    really is.
 
    :param o: the object
    :param ids:
    :return:
    """
    d = deep_getsizeof
    if id(o) in ids:
        return 0
 
    r = sys.getsizeof(o)
    ids.add(id(o))
 
    if isinstance(o, str) or isinstance(0, str):
        return r
 
    if isinstance(o, Mapping):
        return r + sum(d(k, ids) + d(v, ids) for k, v in o.iteritems())
 
    if isinstance(o, Container):
        return r + sum(d(x, ids) for x in o)
 
    return r 

