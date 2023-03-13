import numpy as np
from record3d import Record3DStream
import cv2
from threading import Event
import open3d as ocd


class DemoApp:
    def __init__(self):
        self.event = Event()
        self.session = None
        self.DEVICE_TYPE__TRUEDEPTH = 0
        self.DEVICE_TYPE__LIDAR = 1

    def on_new_frame(self):
        """
        This method is called from non-main thread, therefore cannot be used for presenting UI.
        """
        self.event.set()  # Notify the main thread to stop waiting and process new frame.

    def on_stream_stopped(self):
        print('Stream stopped')

    def connect_to_device(self, dev_idx):
        print('Searching for devices')
        devs = Record3DStream.get_connected_devices()
        print('{} device(s) found'.format(len(devs)))
        for dev in devs:
            print('\tID: {}\n\tUDID: {}\n'.format(dev.product_id, dev.udid))

        if len(devs) <= dev_idx:
            raise RuntimeError('Cannot connect to device #{}, try different index.'
                               .format(dev_idx))

        dev = devs[dev_idx]
        self.session = Record3DStream()
        self.session.on_new_frame = self.on_new_frame
        self.session.on_stream_stopped = self.on_stream_stopped

        self.session.connect(dev)  # Initiate connection and start capturing

    def get_intrinsic_mat_from_coeffs(self, coeffs):
        return np.array([[coeffs.fx,         0, coeffs.tx],
                         [        0, coeffs.fy, coeffs.ty],
                         [        0,         0,         1]])

    def start_processing_stream(self):
        count = 0

        #clear the file
        # f = open('images/pos.txt', "w")
        # f.write("")
        # f.close()

        

        globalArr = []
        
        while count < 200:
            self.event.wait()  # Wait for new frame to arrive

            # Copy the newly arrived RGBD frame
            depth = self.session.get_depth_frame()
            rgb = self.session.get_rgb_frame()
            intrinsic_mat = self.get_intrinsic_mat_from_coeffs(self.session.get_intrinsic_mat())
            print (intrinsic_mat)
            camera_pose = self.session.get_camera_pose()  # Quaternion + world position (accessible via camera_pose.[qx|qy|qz|qw|tx|ty|tz])
            
            poseVec = np.array([[camera_pose.qx],
                       [camera_pose.qy],
                       [camera_pose.qz],
                       [camera_pose.qw]])

            poseVec2 = np.array([[camera_pose.tx],
                       [camera_pose.ty],
                       [camera_pose.tz]])
            

            poseVec = ocd.geometry.get_rotation_matrix_from_quaternion(poseVec)

            pose = np.hstack((poseVec, poseVec2))

            poseFinal = np.vstack((pose, np.array([0, 0, 0, 1])))

            
            
            # f = open('images/pos.txt', "a")
            # f.write(np.array2string(poseFinal))
            # f.write('\n')
            # f.close()

            globalArr.append (poseFinal)


            print(camera_pose.qx)
            # You can now e.g. create point cloud by projecting the depth map using the intrinsic matrix.
        
            # Postprocess it
            if self.session.get_device_type() == self.DEVICE_TYPE__TRUEDEPTH:
                depth = cv2.flip(depth, 1)
                rgb = cv2.flip(rgb, 1)
            
            #print(depth)

            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # Show the RGBD Stream
            cv2.imshow('RGB', rgb)
            cv2.imshow('Depth', depth)
            cv2.waitKey(1)

            # Write the image to the computer
            colorDest = 'frames/color/{}.jpg'.format(count)
            depthDest = 'frames/depth/{}.png'.format(count)
            
            print(f'depth size {np.shape(depth)}')
            
            # Resize the depth image so the dimentions match
            sizeDepth = cv2.resize(depth, (720, 960))
            sizeDepth = sizeDepth * 1000
            depth16 = sizeDepth.astype(np.uint16)
            rgb16 = rgb.astype(np.uint16)
           
            cv2.imwrite(colorDest, rgb16)
            cv2.imwrite(depthDest, depth16)

            count += 1
            #print(intrinsic_mat)
            self.event.clear()
        
        file = np.save("frames/pos.npy", globalArr)


if __name__ == '__main__':
    app = DemoApp()
    app.connect_to_device(dev_idx=0)
    app.start_processing_stream()
