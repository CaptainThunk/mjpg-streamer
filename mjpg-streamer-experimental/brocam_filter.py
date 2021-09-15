
import sys
import cv2
import numpy as np
import datetime
import time
import threading
import pantilthat
import imu01c
from collections import Mapping, Container

class MyFilter:
    

    # kernel for image dilation
    kernel = np.ones((4,4),np.uint8)

    # font style
    font = cv2.FONT_HERSHEY_SIMPLEX

    # directory to save the ouput frames
    pathIn = "/home/pi/debug/"
    pathOut = "/home/pi/Videos/"

    debugMode = True
    debugContours = False
    debugModeWriteImage = False

    #frame_count = 0
    consecutive_frame = 1       # this provides better resolution of a moving object, but the higher the value the more of the moving objects "ghost" it will catch, a larger area covering where it once
   # background_image_count = 0
    setup_initialised = False   # determine if ready to start processing or build up initialisation data
    background_initialised = False # determine if background is available (finsihed in thread)
    background_frames = []      # store for background frames - will use background_frame_count_span for length
    frame_diff_list = []
    background_median_size = 10         # 25
    background_frame_count_span = 50   # Number of frames to save for determining the background (assume approx 5 fps) - should always be higher than background_median_size 150
    background = np.array([])
    o_datetime = 0
    last_background_update_ts = 0
    background_update_time = 120    # in seconds, how often to regenerate the background 300 default (5 mins)
    rebuild_background = False
    is_background_building = False
    
    pthPan = 0
    pthTilt = 0

    recordMode = False
    videoOutput = None
    recordData = True       # records with all added information, False it will be just the raw image
    videoFPS = 5

    # measure fps
    prev_frame_time = 0
    new_frame_time = 0

    # motion thresholds
    frameThreshold = 50
    contourThreshold = 500

    upTime = datetime.datetime.now().timestamp()

    def process(self, img, isRecording, debugCamMode, debugContourMode, motionDetectMode, motionThresholdSlider, contourThresholdSlider):
        '''
            :param img: A numpy array representing the input image
            :param isRecording: An Int sent from output_http indicating whether script should be recording or not
            :param debugCamMode: Enable/Disable debug information
            :param debugContourMode: Enable/Disable contour patterns, for debug purposes
            :param motionDetect: Enable/Disable motion detecting (thereby debugCountourMode as well)
            :returns: A numpy array to send to the mjpg-streamer output plugin
        '''

        # set params sent from plugin
        if(debugCamMode == 1):
            self.debugMode = True
        else:
            self.debugMode = False

        if(debugContourMode == 1):
            self.debugContours = True
        else:
            self.debugContours = False

        if(motionDetectMode == 1):
            self.motionDetect = True
        else:
            self.motionDetect = False

        self.frameThreshold = motionThresholdSlider
        self.contourThreshold = contourThresholdSlider
        
        pthX = pantilthat.get_pan()
        pthY = pantilthat.get_tilt()

        if(pantilthat.get_pan() != self.pthPan or pthY != self.pthTilt):
            self.setup_initialised = False
            self.background_initialised = False
            self.rebuild_background = False
            self.is_background_building = False
            self.background_frames = []
            #self.frame_count = 0
            #self.background_image_count = 0

        self.pthPan = pthX
        self.pthTilt = pthY
        #print("Pan: "+str(self.pthPan))
        #print("Tilt: "+str(self.pthTilt))
#        ori_image = img.copy()

        # get screen dimensions
        img_h = img.shape[0]
        img_w = img.shape[1]
        
        img_w2 = int(img_w/2)
        img_h2 = int(img_h/2)

        valid_cntrs = 0

        # print("Is Recording: "+str(isRecording))
       # self.background = img.copy() # just set up the variable with cur image

        # get datetime object
        self.o_datetime = datetime.datetime.now()

        # recording

        # turn recording on
        if(not self.recordMode and isRecording == 1):
            self.recordMode = True

            self.videoOutput = cv2.VideoWriter(
                self.pathOut+'carcam_'+str(self.o_datetime.strftime("%d-%m-%Y"))+'_'+str(self.o_datetime.strftime("%H-%M-%S"))+'.mp4',
                cv2.VideoWriter_fourcc(*'mp4v'), self.videoFPS,
                (img_w, img_h)

            )

            # write current frame buffer
            for bckFrame in self.background_frames:
                self.videoOutput.write(bckFrame)
    
        # turn recording off
        if(self.recordMode and isRecording == 0):
            self.recordMode = False
            self.videoOutput.release()
        
        # add to file
        if(self.recordMode and not self.recordData):
            self.videoOutput.write(img)
        
        # end recording


        # if not initialised, setup the background image - take x amount of frames and use a random sample of y frames to determine a median image which will remove currently moving objects
        if(self.setup_initialised == False):
            # add image to background_frames list
            self.background_frames.append(img.copy())
            #self.background_image_count = self.background_image_count + 1

            if(self.debugMode):
                print("Debug Initialising Frame: "+str(len(self.background_frames)))

            # if threshold reached do the median calculation and finish initialisation by setting to true
            if(len(self.background_frames) >= self.background_frame_count_span):
                # self.process_background(self.background_median_size)
                self.setup_initialised = True
                self.is_background_building = True
                th = threading.Thread(target=self.process_background)
                th.start()
                
        else:
            if(self.background_initialised and self.motionDetect):
                #self.frame_count = self.frame_count + 1
                
                # remove first frame from background_frames and then add img
                #self.background_frames.pop(0)  # pop causes memoryleak
                del self.background_frames[0]
                self.background_frames.append(img.copy())
                
                #if(self.debugMode):
                #    print("Number of background_frames: "+str(len(self.background_frames)))
                
                # IMPORTANT STEP: convert the frame to grayscale first
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
         
                # find the difference between current frame and base frame
                if(gray.size == self.background.size and self.background.size != 0):
                    frame_diff = cv2.absdiff(gray, self.background)

                    # thresholding to convert the frame to binary
    #                ret, thres = cv2.threshold(frame_diff, 50, 255, cv2.THRESH_BINARY)
                    ret, thres = cv2.threshold(frame_diff, self.frameThreshold, 255, cv2.THRESH_BINARY)
                    # dilate the frame a bit to get some more white area...
                    # ... makes the detection of contours a bit easier
                    dilate_frame = cv2.dilate(thres, None, iterations=1)
                    #if(len(self.frame_diff_list) > 0 and len(self.frame_diff_list) <= self.consecutive_frame):

                    # delete first frame if we have enough consecutive frames
                    if(len(self.frame_diff_list) >= self.consecutive_frame):
                        del self.frame_diff_list[0]
                        #self.frame_diff_list.pop(0)    # pop causes memoryleak

                    # append the final result into the `frame_diff_list`
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

# OLD OLD Method
#                for contour in contours:
#                #for contour in contourList:
#                    # continue through the loop if contour area is less than 500...
#                    # ... helps in removing noise detection
#                    if cv2.contourArea(contour) < self.contourThreshold:
#                        continue
#                    # get the xmin, ymin, width, and height coordinates from the contours
#                    (x, y, w, h) = cv2.boundingRect(contour)
#                    # draw the bounding boxes
#                    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
#                    valid_cntrs = valid_cntrs + 1
#        #            cv2.imshow('Detected Objects', orig_frame)

#

# OLD Method

                # # draw the contours, not strictly necessary
                # if(self.debugContours):
                #     for i, cnt in enumerate(contours):
                #         cv2.drawContours(img, contours, i, (255, 0, 0), 3)

                # # go through contours and find boxes that overlap
                # contourList = []
                # excludeList = []

                # for i, contour in enumerate(contours):
                #     if cv2.contourArea(contour) < self.contourThreshold:
                #         continue

                #     if i not in excludeList:
                #         (x, y, w, h) = cv2.boundingRect(contour)

                #         for i2, contour_2 in enumerate(contours):
                #             if(i == i2):    # skip self
                #                 continue

                #             if cv2.contourArea(contour_2) < self.contourThreshold:
                #                 continue

                #             if i2 in excludeList:
                #             #    print("Skip Excluded")
                #                 continue


                #             (x2, y2, w2, h2) = cv2.boundingRect(contour_2)

                #             # test if overlap
                #             if( 
                #                 ((x2 >= x and y2 >= y) and (x2 <= x+w and y2 <= y+h)) or
                #                 ((x2+w2 >= x and y2+h2 >= y ) and (x2+w2 <= x+w and y2+h2 <= y+h)) or
                #                 ((x2 >= x and y2 >= y) and (x2+w2 <= x+w and y2+h2 <= y+h)) or
                #                 ((x2+w2 >= x and y2+h2 >= y ) and (x2 <= x+w and y2 <= y+h)) 
                #             ):
                #                 #print("Overlap - X:"+str(x)+", y:"+str(y)+", w:"+str(w)+", h:"+str(h)+" -- x2:"+str(x2)+", y2:"+str(y2)+", w2:"+str(w2)+", h2:"+str(h2))

                #                 #overlaps so set countour and countour_2 to exclude - no longer want to check it as we now know its an overlapper
                #                 if i not in excludeList:
                #                     excludeList.append(i)
                #                 if i2 not in excludeList:
                #                     excludeList.append(i2)

                #                 newX = x
                #                 newY = y
                #                 newW = x+w
                #                 newH = y+h

                #                 if(x2 < x):
                #                     newX = x2
                #                 if(y2 < y):
                #                     newY = y2
                #                 if(x2+w2 > newW):
                #                     newW = x2+w2
                #                 if(y2+h2 > newH):
                #                     newH = y2+h2

                #                 # append to new countourlist with max value x,y,w,h
                #                 #contourList.append([newX, newY, newX-newW, newY-newH])
                #                 #np.append(contourList, [newX, newY, newX-newW, newY-newH], axis=0)

                #                 cv2.rectangle(img, (newX, newY), (newW, newH), (0, 0, 255), 2)
                #                 valid_cntrs = valid_cntrs + 1

                #     if i not in excludeList:
                #         cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                #         valid_cntrs = valid_cntrs + 1
                #      #   np.append(contourList, [x, y, w, h], axis=0)


# New method for motion detection bounding boxes. This merges all overlapping boxes.             
                # draw the contours, not strictly necessary - move to after threshold check if want to show only ones acted upon (change the -1 to i though so it doesn't do all each iteration)
                if(self.debugContours):
                    cv2.drawContours(img, contours, -1, (255, 0, 0), 2)
               
                        
                # go through contours and find boxes that overlap
                contourList = []
                finalContourList = []
                #excludeList = []
                #contourDimensions = []

                for i, contour in enumerate(contours):
                    if cv2.contourArea(contour) < self.contourThreshold:
                        continue

                    contourList.append(cv2.boundingRect(contour));

# TEMP DEBUG
                # Red ordinary contours (debug purposes)
                # for (x, y, w, h) in contourList:
                #     cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)

                # Interate the contourlist until there are no more overlaps using function
                while True:
                    finalContourList = findOverlappingContours(contourList)
                    #if((contourList is None and finalContourList is None) or set(contourList) == set(finalContourList)):
                    if(set(contourList) == set(finalContourList)):
                        break
                    contourList = finalContourList


                # Green overlaps and overwrite nooverlaps
                for (x, y, w, h) in finalContourList:
                    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    valid_cntrs += 1


                    #print(str(i)+": x="+str(x)+", y="+str(y)+", w="+str(w)+", h="+str(y) )
                    # for i2, (x2, y2, w2, h2) in enumerate(contourList):
                    #     if(i == i2):    # skip self
                    #         continue




                    # if i not in excludeList:
                    #     (x, y, w, h) = cv2.boundingRect(contour)

                    #     for i2, contour_2 in enumerate(contours):
                    #         if(i == i2):    # skip self
                    #             continue

                    #         if cv2.contourArea(contour_2) < self.contourThreshold:
                    #             continue

                    #         if i2 in excludeList:
                    #         #    print("Skip Excluded")
                    #             continue


                    #         (x2, y2, w2, h2) = cv2.boundingRect(contour_2)

                    #         # test if overlap
                    #         if( 
                    #             ((x2 >= x and y2 >= y) and (x2 <= x+w and y2 <= y+h)) or
                    #             ((x2+w2 >= x and y2+h2 >= y ) and (x2+w2 <= x+w and y2+h2 <= y+h)) or
                    #             ((x2 >= x and y2 >= y) and (x2+w2 <= x+w and y2+h2 <= y+h)) or
                    #             ((x2+w2 >= x and y2+h2 >= y ) and (x2 <= x+w and y2 <= y+h)) 
                    #         ):
                    #             #print("Overlap - X:"+str(x)+", y:"+str(y)+", w:"+str(w)+", h:"+str(h)+" -- x2:"+str(x2)+", y2:"+str(y2)+", w2:"+str(w2)+", h2:"+str(h2))

                    #             #overlaps so set countour and countour_2 to exclude - no longer want to check it as we now know its an overlapper
                    #             if i not in excludeList:
                    #                 excludeList.append(i)
                    #             if i2 not in excludeList:
                    #                 excludeList.append(i2)

                    #             newX = x
                    #             newY = y
                    #             newW = x+w
                    #             newH = y+h

                    #             if(x2 < x):
                    #                 newX = x2
                    #             if(y2 < y):
                    #                 newY = y2
                    #             if(x2+w2 > newW):
                    #                 newW = x2+w2
                    #             if(y2+h2 > newH):
                    #                 newH = y2+h2

                    #             # append to new countourlist with max value x,y,w,h
                    #             #contourList.append([newX, newY, newX-newW, newY-newH])
                    #             #np.append(contourList, [newX, newY, newX-newW, newY-newH], axis=0)

                    #             cv2.rectangle(img, (newX, newY), (newW, newH), (0, 0, 255), 2)
                    #             valid_cntrs = valid_cntrs + 1

                    # if i not in excludeList:
                    #     cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    #     valid_cntrs = valid_cntrs + 1
                    #  #   




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
#                    if (cv2.contourArea(cntr) >= self.contourThreshold):
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
        if(self.setup_initialised and self.background_initialised and self.debugMode and self.motionDetect):
            cv2.putText(img, "motions detected: " + str(valid_cntrs), (55, 15), self.font, 0.6, (0, 180, 0), 2)

        if(self.is_background_building):
            cv2.putText(img, "Building background", (55, 36), self.font, 0.6, (0, 180, 0), 2)

        # crude crosshair
        if(self.debugMode):
            crosshair_size = 16 # bigger the number the smaller the cross hair
            cv2.line(img, (int(img_w2 - (img_w/crosshair_size)), img_h2), (int(img_w2 + (img_w/crosshair_size)), img_h2), (0xff, 0, 0), thickness=3)
            cv2.line(img, (img_w2, (int(img_h2 - (img_h/crosshair_size)))), (img_w2, int(img_h2 + (img_h/crosshair_size))), (0xff, 0, 0), thickness=3)

        # Info box
        margin = 20
        numTextLines = 7 # doesn't include Title
        fontscale = 0.6

        # do time calc as late as possible
        # time when we finish processing for this frame
        self.new_frame_time = time.time()
        fps = 1/(self.new_frame_time-self.prev_frame_time)
        self.prev_frame_time = self.new_frame_time
        self.videoFPS = fps

        # Info box - text strings
        titleText = "BroNet Car Cam"
        dateText = self.o_datetime.strftime("%d-%m-%Y")
        timeText = self.o_datetime.strftime("%X")
        panText = "Cam Pan Angle: "+str(self.pthPan)
        tiltText = "Cam Tilt Angle: "+str(self.pthTilt) 
        upTimeText = "Uptime: "+str(datetime.datetime.fromtimestamp(datetime.datetime.utcnow().timestamp() - self.upTime).strftime("%X"))
        # temperatureText = imu01c.getL3GD20HTemp()   # these are the 2 sensors for temperature on the imu01c - choose one
        temperatureText = imu01c.getLSM303DTemp()+12   # these are the 2 sensors for temperature on the imu01c - choose one

        # temperatureText = "{0:6.2f}".format(temperatureText)+"&#8451;C"
        # temperatureText = "{0:6.2f}".format(temperatureText)+"°C"
        temperatureText = "{0:6.2f}".format(temperatureText)+"C"

        # set dateText to new value of time date temp
        dateText = timeText+" "+dateText+" "+temperatureText

        # Info box - calculate longest string to determine positioning
        textsize_titletext = cv2.getTextSize(titleText, self.font, 1, 2)[0]
        textsize_dateText = cv2.getTextSize(dateText, self.font, fontscale, 1)[0]
        textsize_timeText = cv2.getTextSize(timeText, self.font, fontscale, 1)[0]
        textsize_pantext = cv2.getTextSize(panText, self.font, fontscale, 1)[0]
        textsize_tilttext = cv2.getTextSize(tiltText, self.font, fontscale, 1)[0]
        textsize_uptimetext = cv2.getTextSize(upTimeText, self.font, fontscale, 1)[0]

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

        # see if uptimetext is bigger
        if(textsize_uptimetext[0] > textX):
            textX = textsize_uptimetext[0]

        if(textsize_uptimetext[1] > textY):
            textY = textsize_uptimetext[1]

        posTextX = img_w - textX - margin
        posTextY = img_h - (textY * numTextLines) - margin     # set size of box based on biggest textY * number of text lines

        # Info box - write text to image
        cv2.putText(img, titleText, (posTextX, posTextY - textsize_titletext[1]), self.font, 1, (93, 93, 216), 2)
        cv2.putText(img, dateText, (posTextX, posTextY), self.font, fontscale, (0, 180, 0), 1)
        # cv2.putText(img, timeText, (posTextX, posTextY + textY), self.font, fontscale, (0, 180, 0), 1)
        cv2.putText(img, panText, (posTextX, posTextY + (textY*1)), self.font, fontscale, (0, 180, 0), 1)
        cv2.putText(img, tiltText, (posTextX, posTextY + (textY*2)), self.font, fontscale, (0, 180, 0), 1)
        cv2.putText(img, upTimeText, (posTextX, posTextY + (textY*3)), self.font, fontscale, (0, 180, 0), 1)
        cv2.putText(img, "FPS: "+str(int(fps)), (posTextX, posTextY + (textY*4)), self.font, fontscale, (0, 180, 0), 1)
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
                dbgCountdown = self.background_update_time - int(datetime.datetime.now().timestamp()-self.last_background_update_ts)
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
        if(self.last_background_update_ts != 0 and (self.o_datetime.timestamp() - self.last_background_update_ts) > self.background_update_time and self.setup_initialised == True):
            self.rebuild_background = True

            initMsg = "Rebuilding Background for Object Motion Detection"
            textsize_initMsg = cv2.getTextSize(initMsg, self.font, 1, 2)[0]
            initMsgX = textsize_initMsg[0]
            initMsgY = textsize_initMsg[1]

            # center text
            centerTextX = img_w2 - int(initMsgX / 2)
            centerTextY = img_h2 - int(initMsgY / 2)
            cv2.putText(img, initMsg, (centerTextX, centerTextY), self.font, 1, (200, 200, 200), 2)

        # add to file with data
        if(self.recordMode and self.recordData):
            self.videoOutput.write(img)

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
        if(self.debugMode):
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
        if(self.debugMode):
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


def findOverlappingContours(contours):
    '''
        This function is takes a list and finds overlaps and returns the new list 
    '''
    newContourList = []
    excludeList = []

    # iterate contours list
    for i, (x, y, w, h) in enumerate(contours):
        # debug output
        #print(str(i)+": x="+str(x)+", y="+str(y)+", w="+str(w)+", h="+str(y) )

        # if isn't excluded (eg, already been handled as an overlap)
        if i not in excludeList:
            for i2, (x2, y2, w2, h2) in enumerate(contours):
                if(i == i2):    # skip self
                    continue

                # skip if already handled and added to excludeList
                if i2 in excludeList:
                    #    print("Skip Excluded")
                    continue

                # Check if boxes overlap
                if( 
                    ((x2 >= x and y2 >= y) and (x2 <= x+w and y2 <= y+h)) or
                    ((x2+w2 >= x and y2+h2 >= y ) and (x2+w2 <= x+w and y2+h2 <= y+h)) or
                    ((x2 >= x and y2 >= y) and (x2+w2 <= x+w and y2+h2 <= y+h)) or
                    ((x2+w2 >= x and y2+h2 >= y ) and (x2 <= x+w and y2 <= y+h)) 
                ):

                    # debug output
                    #print("Overlap - X:"+str(x)+", y:"+str(y)+", w:"+str(w)+", h:"+str(h)+" -- x2:"+str(x2)+", y2:"+str(y2)+", w2:"+str(w2)+", h2:"+str(h2))
        
                    # overlaps so exclude both i and i2 - no longer want to check it as we now know its an overlapper and has been handled
                    if i not in excludeList:
                        excludeList.append(i)
                    if i2 not in excludeList:
                        excludeList.append(i2)

                    # define new box using max values of both (probably could use MAX() MIN() type functions here)
                    newX = x
                    newY = y
                    newW = x+w
                    newH = y+h

                    if(x2 < x):
                        newX = x2
                    if(y2 < y):
                        newY = y2
                    if(x2+w2 > newW):
                        newW = x2+w2
                    if(y2+h2 > newH):
                        newH = y2+h2

                    # append to new countourlist with max value x,y,w,h
                    newContourList.append((newX, newY, newW-newX, newH-newY))

        # if not in excludelist because it didn't overlap add to list as ordinary box
        if i not in excludeList:
            newContourList.append((x, y, w, h))

    return newContourList