import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


class Lane:
    def __init__(self):
        self.detected = False

        # polynomial coefficients averaged over the last n iterations
        self.best_fit = None
        # polynomial coefficients for the most recent fit
        self.current_fit = None

        # radius of curvature of the line in some units
        self.best_curvature = None
        self.curvature = None
        # distance in meters of vehicle center from the line
        self.best_offset = None
        self.offset = None

        # difference between this line and the new detected line
        self.diff = np.array([0, 0, 0], dtype='float')


class LaneDetection:
    def __init__(self):
        self.left_line = Lane()
        self.right_line = Lane()

        self._M_inv = None

        self.curvature = None
        self.offset = None
        self.result_image = None

    def perspective_transform(self, img):
        x_size = img.shape[1]
        y_size = img.shape[0]

        offset = 200
        # Origin points
        src = np.float32(
            [[599, y_size * 0.55],
            [688, y_size * 0.55],
            [1025, y_size * 1],
            [165, y_size * 1]])
        # Destination points
        dst = np.float32(
            [[offset, 0],
            [x_size - offset, 0],
            [x_size - offset, y_size],
            [offset, y_size]])

        # Calculate transform matrix and inverse transform matrix
        trans_mat = cv2.getPerspectiveTransform(src, dst)
        self._M_inv = cv2.getPerspectiveTransform(dst, src)

        # Warp image
        warped = cv2.warpPerspective(img, trans_mat, img.shape[1::-1])

        return warped

    @staticmethod
    def lane_edge_detection(img):
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        # Define threshold
        mag_thresh = (7, 255)
        dir_thresh = (0, np.pi / 2 / 90 * 35)
        s_thresh = (50, 255)
        r_thresh = (190, 255)
        gray_thresh = (232, 255)

        # Prepare Sobel and HLS images
        sobel_x = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3))
        sobel_y = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3))
        mag = np.sqrt(sobel_x ** 2 + sobel_y ** 2)
        dire = np.absolute(np.arctan2(sobel_y, sobel_x))
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        s_channel = hls[:, :, 2]
        r_channel = img[:, :, 0]

        # Create a mask
        # ((White & Yellow) & (mag_exclude_weak) & (dir_verticle)) | (White_supplement)
        mask = (((r_channel >= r_thresh[0]) & (r_channel <= r_thresh[1]) |
                (s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])) &
                ((mag >= mag_thresh[0]) & (mag <= mag_thresh[1])) &
                ((dire >= dir_thresh[0]) & (dire <= dir_thresh[1])) |
                ((gray >= gray_thresh[0]) & (gray <= gray_thresh[1])))

        # Prepare an image to binarize
        binary = np.zeros_like(gray)
        binary[mask] = 255

        return binary

    @staticmethod
    # Apply sliding windows technique to fit both lines in a image without prior found lines
    def sliding_windows(image):
        # Using histogram to locate appropriate line position
        histogram = np.sum(image, axis=0)
        midpoint = np.int(image.shape[1] // 2)
        left_base = np.argmax(histogram[:midpoint - 200])
        right_base = np.argmax(histogram[midpoint + 200:]) + midpoint + 200

        # Set up windows
        no_windows = 9  # number
        margin = 70  # width
        min_pix = 50  # minimum number of pixels

        # Candidate pixels
        window_height = np.int(image.shape[0] // no_windows)
        nonzero = image.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])

        # Pointer and contatiner
        left_current = left_base
        right_current = right_base
        left_lane_inds = []
        right_lane_inds = []

        # Track curvature
        for window in range(no_windows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = image.shape[0] - (window + 1) * window_height
            win_y_high = image.shape[0] - window * window_height
            win_xleft_low = left_current - margin // 2
            win_xleft_high = left_current + margin // 2
            win_xright_low = right_current - margin // 2
            win_xright_high = right_current + margin // 2

            # Indicate pixels in current window
            good_right_inds = ((nonzero_y <= win_y_high) & (nonzero_y >= win_y_low) &
                            (nonzero_x >= win_xright_low) & (nonzero_x <= win_xright_high)).nonzero()[0]
            good_left_inds = ((nonzero_y <= win_y_high) & (nonzero_y >= win_y_low) &
                            (nonzero_x >= win_xleft_low) & (nonzero_x <= win_xleft_high)).nonzero()[0]
            # Update window position
            if len(good_right_inds) > min_pix:
                right_current = np.int(np.mean(nonzero_x[good_right_inds]))
            if len(good_left_inds) > min_pix:
                left_current = np.int(np.mean(nonzero_x[good_left_inds]))
            right_lane_inds.append(good_right_inds)
            left_lane_inds.append(good_left_inds)

            # cv2.rectangle(image, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (255, 255, 255), 4)
            # cv2.rectangle(image, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (255, 255, 255), 4)

        # Fit a polynomial
        # Acquire points
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzero_x[left_lane_inds]
        lefty = nonzero_y[left_lane_inds]
        rightx = nonzero_x[right_lane_inds]
        righty = nonzero_y[right_lane_inds]

        left_fit = np.polyfit(lefty, leftx, 1)
        right_fit = np.polyfit(righty, rightx, 1)

        return [0, left_fit[0], left_fit[1]], [0, right_fit[0], right_fit[1], 0]

    # Fit lines in images with prior detected lines
    # Return both fit information in pixel and in meter
    def fit_from_prior(self, image, left_fit, right_fit):
        # Set margin
        margin = 50

        # Grab activated pixels
        nonzero = image.nonzero()
        nonzero_x = nonzero[1]
        nonzero_y = nonzero[0]

        # Set search area
        left_lane_inds = abs((left_fit[0] * nonzero_y ** 2 + left_fit[1] * nonzero_y + left_fit[2]) - nonzero_x) < margin
        right_lane_inds = abs(
            (right_fit[0] * nonzero_y ** 2 + right_fit[1] * nonzero_y + right_fit[2]) - nonzero_x) < margin

        # Exact left and right line pixels (in pixel)
        leftx = nonzero_x[left_lane_inds]
        lefty = nonzero_y[left_lane_inds]
        rightx = nonzero_x[right_lane_inds]
        righty = nonzero_y[right_lane_inds]

        # Define conversions in x and y from pixels space to meters
        ym_per_pix = 30.0 / 720
        xm_per_pix = 3.7 / 700
        # Change from pixels space to meter
        leftx_cr = leftx * xm_per_pix
        lefty_cr = lefty * ym_per_pix
        rightx_cr = rightx * xm_per_pix
        righty_cr = righty * ym_per_pix

        return (self.fit_poly(leftx, lefty, rightx, righty), 
                self.fit_poly(leftx_cr, lefty_cr, rightx_cr, righty_cr))

    @staticmethod
    # Helper function to fit both lines with selected pixels
    def fit_poly(leftx, lefty, rightx, righty):

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        return left_fit, right_fit

    # Calculates the curvature of polynomial functions in meters.
    def measure_curvature(self, image, left_fit_cr, right_fit_cr):
        # Define conversions in x and y from pixels space to meters
        ym_per_pix = 30 / 720
        xm_per_pix = 3.7 / 700

        # Image size
        y_eval = image.shape[0]

        # Implement the calculation of the left line here
        left_curverad = (1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** (3 / 2) / (
                    2 * abs(left_fit_cr[0]))
        # Implement the calculation of the right line here
        right_curverad = (1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** (3 / 2) / (
                    2 * abs(right_fit_cr[0]))

        self.left_line.curvature = left_curverad
        self.right_line.curvature = right_curverad

    # Calculates the offset of polynomial functions in meters.
    def measure_offset(self, image):
        # Define conversions in x and y from pixels space to meters
        ym_per_pix = 30 / 720
        xm_per_pix = 3.7 / 700
        
        # Image information
        y_eval = image.shape[0]
        mid_point = image.shape[1] // 2

        # Calculate lines position
        left_fitp = np.dot(self.left_line.current_fit, np.array([y_eval ** 2, y_eval, 1]))
        right_fitp = np.dot(self.right_line.current_fit, np.array([y_eval ** 2, y_eval, 1]))

        # Calculate offset
        left_offset = (mid_point - left_fitp) * xm_per_pix
        right_offset = (right_fitp - mid_point) * xm_per_pix

        self.left_line.offset = left_offset
        self.right_line.offset = right_offset

    def check_lines(self):
        # Defina a flag
        are_lines = True

        # Check curvature
        # allowed difference between two curvatures is distant from distance to distance
        # 0~300: 3 times, 300~700: 5 times, 700+: no limit
        if not (0.2 < self.left_line.curvature / self.right_line.curvature < 5):
            if self.left_line.curvature < 700 or self.right_line.curvature < 700:
                print('curvature error')
                print(self.left_line.curvature, self.right_line.curvature)
                are_lines = False
        if not (0.33 < self.left_line.curvature / self.right_line.curvature < 3):
            if self.left_line.curvature < 300 or self.right_line.curvature < 300:
                print('curvature error')
                print(self.left_line.curvature, self.right_line.curvature)
                are_lines = False

        # Check offset
        # Offset should not be too distant from each other since the cat stays in the center
        if abs(self.left_line.offset - self.right_line.offset) > 1.2:
            print('offset error')
            print(self.left_line.offset, self.right_line.offset)
            are_lines = False

        # All tests pass
        if are_lines:
            # If last lines exist
            if self.left_line.detected:
                self.left_line.best_curvature = self.left_line.curvature
                self.right_line.best_curvature = self.right_line.curvature
                self.left_line.best_offset = self.left_line.offset
                self.right_line.best_offset = self.right_line.offset

                # Smooth the change process
                self.left_line.best_fit += (self.left_line.current_fit - self.left_line.best_fit) / 5
                self.right_line.best_fit += (self.right_line.current_fit - self.right_line.best_fit) / 5

            # If last lines don't exist
            else:
                self.left_line.detected = True
                self.right_line.detected = True
                self.left_line.best_fit = self.left_line.current_fit
                self.right_line.best_fit = self.right_line.current_fit
                self.left_line.best_curvature = self.left_line.curvature
                self.right_line.best_curvature = self.right_line.curvature
                self.left_line.best_offset = self.left_line.offset
                self.right_line.best_offset = self.right_line.offset

    # Fit polynomial lines in the image and calculate relevent curvature and offset
    def fit_and_cal_curvature_offset(self, image):
        # Load prior fit data, if there isn'y, apply a sliding windows technique
        if not self.left_line.detected:
            left_fit, right_fit = self.sliding_windows(image)
        else:
            left_fit, right_fit = self.left_line.best_fit, self.right_line.best_fit

        # Fit from prior lines to update line information
        [self.left_line.current_fit, self.right_line.current_fit], [left_fit_cur, right_fit_cur] = \
         self.fit_from_prior(image, left_fit, right_fit)

        # Calculate curvature and offset of two lines
        self.measure_curvature(image, left_fit_cur, right_fit_cur)
        self.measure_offset(image)

        # To check if new lines detection are valid, if they are, replace the detection with current one in a smooth process
        self.check_lines()

        # If lines are not found yet
        if not self.left_line.detected:
            image_zero = np.zeros_like(image).astype(np.uint8)
            return np.dstack((image_zero, image_zero, image_zero)), [0, 0], 0

        # Acquire final output of this frame
        left_fit = self.left_line.best_fit
        right_fit = self.right_line.best_fit
        curvature = (self.left_line.best_curvature + self.right_line.best_curvature) / 2
        offset = (self.right_line.best_offset - self.left_line.best_offset) / 2

        # Visualization
        # Create an image to draw the lines on
        image_zero = np.zeros_like(image).astype(np.uint8)
        lane_image = np.dstack((image_zero, image_zero, image_zero))

        # Exact x and y points of two lines
        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0], dtype=np.int_)
        left_fitx = np.int_(np.clip(left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2], 0, image.shape[1] - 1))
        right_fitx = np.int_(
            np.clip(right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2], 0, image.shape[1] - 1))

        # Draw the lane onto the image
        lane_image[ploty, left_fitx, :] = [255, 0, 0]
        lane_image[ploty, right_fitx, :] = [255, 0, 0]
        # Draw the allowed area onto the image
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        cv2.fillPoly(lane_image, pts, (0, 255, 0))

        self.curvature = curvature
        self.offset = offset

        return lane_image

    def unwarp_found_region(self, undist_image, lane_image):
        # Warp the lane image to original image space using inverse perspective matrix (Minv)
        unwarp_lane = cv2.warpPerspective(lane_image, self._M_inv, (undist_image.shape[1], undist_image.shape[0]))

        # Combine the result with the original image
        result = cv2.addWeighted(undist_image, 1, unwarp_lane, 0.3, 0)

        # Print out both lines' curvature and offset
        if self.curvature > 100:
            cur_text = "Straight line ahead"
        else:
            cur_text = "Radius of Curvature: {}(m)".format(int(self.curvature))
        if self.offset > 0:
            offset_text = "Vehicle is {:.2f}m left of center".format(self.offset)
        else:
            offset_text = "Vehicle is {:.2f}m right of center".format(-self.offset)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(result, cur_text, (450, 70), font, 0.8, (0, 255, 255), 2)
        cv2.putText(result, offset_text, (450, 120), font, 0.8, (0, 255, 255), 2)
        return result

    def lane_detection(self, img):
        # 1, Undistort image
        # Camera distortion is ignored

        # 2,Apply Perspective transform
        warped = self.perspective_transform(img)

        # 3,Apply Lane edges detection
        binary = self.lane_edge_detection(warped)
        
        # 4,Fit polynomial equation, calculate curvature and offset
        lane_image = self.fit_and_cal_curvature_offset(binary)

        # 5,Draw final image
        self.result_image = self.unwarp_found_region(img, lane_image)


if __name__ == "__main__":
    image_path = "test.png"
    image = cv2.imread(image_path)
    image = image[:, :, ::-1]

    ld = LaneDetection()
    ld.lane_detection(image)
    result = ld.result_image

    plt.imshow(result)
    plt.show()
