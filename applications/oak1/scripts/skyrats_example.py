#optical flow
import numpy as np
import cv2



cap = cv2.VideoCapture(0)


class CameraStabilization():

    feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15,15),
                      maxLevel = 2,
                      criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    def __init__(self,flip = True):

        self.flip = flip

        # Create some random colors
        self.color = np.random.randint(0,255,(100,3))
        
    def resetFeatures(self, initial_img):

        self.display_mask = np.zeros_like(initial_img)                #mask for movement lines
        self.old_gray = cv2.cvtColor(initial_img, cv2.COLOR_BGR2GRAY) #coverts to gray scale
        
        #detect features in initial_img to track
        #p0 contains the last frame feature positions(x,y) - p0.shape = (number of features, 1, 2) 
        self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)
        
    def calculateCameraDrift(self, frame, feature_params=feature_params, show=True,  precision_cut = 10):

        movement = np.zeros(2)
        precise = True             #becomes false if the standard deviation > precision_cut
        
        if self.flip:
            frame = cv2.flip(frame,1)

        if not hasattr(self, 'p0'):    #checks if "p0" is defined
            self.resetFeatures(frame)  # if not, reset features to define this array
            print("reset")
            Precise = False
        else:

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #p1 contains p0 feature on the new frame positions(x,y) 
            p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray,frame_gray, self.p0, None, **self.lk_params)
            
            if(type(p1) == 'NoneType'):
                print("frame contains no fetures")
                self.resetFeatures(frame)
                return

            good_new = p1[st==1]        #select only usable features
            good_old = self.p0[st==1]

            if show:
                self.display(frame,good_new,good_old)

            dist_movement = good_new-good_old
            
            if(dist_movement.shape[0]>1 and dist_movement.shape[1]>1):

                #remove outliers dist_movements
                dist_movement_accurate, total_std = self.cutOutliers(dist_movement, num_std = 1.2)

                movement  = np.mean(dist_movement_accurate,0)

                self.old_gray = frame_gray.copy()  #update reference gray image
                self.p0 = good_new.reshape(-1,1,2) #update the list of reference points on track (features locations)
                
                if(total_std > precision_cut):
                    precise = False
                else:
                    precise = True
            else:
                self.resetFeatures(frame)  #forces to have more than 1 feature on the screen

        return movement, precise  #return the imagem movement in pixels and if this value ir precise or not

    def cutOutliers(self, list, num_std = 2):

        list_mean = np.mean(list,0) 
        std = np.std(list,0)          #standard deviation
        outlier_cut = num_std*std

        total_std = np.average(std)   #used to validate or not the frame movement
        #print(total_std)
        
        binary_mask_up = list<=list_mean+outlier_cut+0.1        #boolean array to exclude random positive movements
        binary_mask_down = list>=list_mean-outlier_cut-0.1      #boolean array to exclude random negative movements

        mask_measure = binary_mask_up & binary_mask_down        #boolean array of good movements (on each axis) to consider

        #boolean array that leaves only measures with good movements on both axis 
        # equivalent to: an and condition for the row
        mask_outlier = np.min(mask_measure,1)           
        

        return list[mask_outlier], total_std

    def display(self,frame,good_new,good_old):   #draw stuff

        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel()
            c,d = old.ravel()

            self.display_mask = cv2.line(self.display_mask, (a,b),(c,d), self.color[i].tolist(), 2)
            frame = cv2.circle(frame,(a,b),5,self.color[i].tolist(),-1)
            
        img = cv2.add(frame,self.display_mask)
        cv2.imshow('frame',img)

        return frame

    def clearMask(self):
        self.display_mask[:] = 0
        return



def main():

    c = CameraStabilization()
    X, Y = 200,200                 #initial window position
    #w, h = (250,250)               #initial window dimensions

    img_drone = True
    drone = cv2.imread("3dr-drone4.png",-1)  #image to be displayed
    h, w,_ = drone.shape

    w = int(w/5)
    h= int(h/5)
    drone = cv2.resize(drone,(w,h))          #reduces the image

    rows,cols,_ = drone.shape

    while(1):
        ret,frame = cap.read()
        
        move, precise = c.calculateCameraDrift(frame, precision_cut = 1)
        X += move[0]
        Y += move[1]
        print(precise)
        frame = cv2.flip(frame,1)

        img = frame[:]
        
        if(img_drone):

            Y_on_screen = min(max(Y, 0),frame.shape[1])
            X_on_screen = min(max(X, 0),frame.shape[0])

            drone=drone[int(Y_on_screen-np.abs(Y)):,int(X_on_screen-np.abs(X)):]
            ch = cv2.split(drone); #ch[3] = background mask of the drone
            print(drone.shape)
            print(ch[3].shape)

            

            
            roi = img[int(Y_on_screen):int(Y+h),int(X_on_screen):int(X+w),:]
            print(roi.shape)

            mask_inv = cv2.bitwise_not(ch[3])
            # Black-out the area of drone in ROI
            frame_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)

            # Take only region of drone body.
            drone_fg = cv2.bitwise_and(drone,drone,mask = ch[3])

            # Put logo in ROI and modify the main image
            dst = cv2.add(frame_bg,drone_fg[:,:,:3])
            img[int(Y):int(Y+h),int(X):int(X+w),:]  = dst

        else:
            img = cv2.rectangle(img,(int(X),int(Y)),(int(X+w),int(Y+h)),(0,255,0),3)


        cv2.imshow('res',img)  #display result



        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
        elif (k == ord('e')):
            print("erase screen")
            c.clearMask()  #clears all line
        elif (k == ord('r')):
            print("reset features")
            c.resetFeatures(frame)

        #moves the drone default position
        elif (k == ord('w')):
            Y-=5
        elif (k == ord('s')):
            Y+=5
        elif (k == ord('a')):
            X-=5
        elif (k == ord('d')):
            X+=5

        elif (k == ord('m')):
            print("change image")
            img_drone = not img_drone



    cv2.destroyAllWindows()
    cap.release()

if __name__ == "__main__":
    main()
