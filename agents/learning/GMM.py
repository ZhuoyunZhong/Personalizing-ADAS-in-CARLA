#!/usr/bin/env python
# coding=utf-8

import pickle
from os import path
from os.path import dirname, abspath

from sklearn.cluster import KMeans
import scipy
from scipy.stats import multivariate_normal
import numpy as np 


# Gaussian Mix Model
# [V, H, DL, DF] -- GMM --> [t_lat]

class GMM:
    def __init__(self):
        self._data_folder = path.join(dirname(dirname(dirname(abspath(__file__)))), "data")
        self._train_file_path = path.join(self._data_folder, "GMM_train.pickle")
        self._model_file_path = path.join(self._data_folder, "GMM_model.pickle")

        self._Gaussian_set_num = 6
        self._data_dim = None
        self._iteration_t = 1
        self._p = None
        self._mean = None
        self._covar = None
        self._GMM_model = None

        self._train_data = None
        self._loss = 0
        self._import_model()

    def _import_model(self):
        # if file doesn't exist, train the model
        if not path.exists(self._model_file_path):
            print('GMM model does not exist')
            if path.exists(self._train_file_path):
                with open(self._train_file_path, 'rb') as f:
                    self._train_data = pickle.load(f)
                self.train(np.array(self._train_data), self._Gaussian_set_num, self._iteration_t)
            else:
                with open(self._train_file_path, 'wb') as f:
                    self._train_data = [[8.0, -3.5, 15.0, -10.0, 5.0]]
                    pickle.dump(self._train_data, f)
                self.train(np.array(self._train_data), self._Gaussian_set_num, self._iteration_t)
                
        # Load GMM model
        with open(self._model_file_path, 'rb') as f:
            self._GMM_model = pickle.load(f)
            self._p = self._GMM_model['p']
            self._mean = self._GMM_model['mean']
            self._covar = self._GMM_model['covariance']
        print('Loaded GMM model')
    
    def _initial_GMM_params(self, data):
        # K-means cluster
        data_size, dim = data.shape[0], data.shape[1]
        self._p = np.zeros(self._Gaussian_set_num)
        self._mean = np.zeros((self._Gaussian_set_num, dim))
        self._covar = np.zeros((self._Gaussian_set_num, dim, dim))

        kmeans = KMeans(n_clusters=self._Gaussian_set_num).fit(data)
        for label in range(self._Gaussian_set_num):
            points_ids = np.nonzero(label == kmeans.labels_)[0] # 1d

            # initial p, mean and covariance
            self._p[label] = float(len(points_ids)) / data_size
            self._mean[label, :] = np.mean(data[points_ids], axis=0)
            diff_mean = data[points_ids] - self._mean[label, :]
            self._covar[label, :, :] = self._p[label] * np.dot(diff_mean.T, diff_mean) \
                                         / len(points_ids)
            
    def train(self, data=None, Gaussian_set_num=None, iteration_t=None):
        if Gaussian_set_num:
            self._Gaussian_set_num = Gaussian_set_num
        if iteration_t:
            self._iteration_t = iteration_t

        data_size, dim = data.shape[0], data.shape[1]
        self._data_dim = dim

        # EM Algorithm
        # Initial guess with k-means cluster
        self._initial_GMM_params(data)

        # Run EM
        for t in range(iteration_t):
            # E-step
            '''
            Q = Sum(i=1:N)( Sum(zi=z1:zK)(log`p(xi,zi|theta) × p(zi|xi,theta_t)) )
            Q = Sum(k=1:K)( Sum(i=1:N)( (log`pk + log`N(xi|meank,covark) x p(zi_t|xi,theta_t)) )
            '''
            # compute post probability: p(zi_t|xi,theta_t)
            gamma = np.zeros((data_size, self._Gaussian_set_num))

            for label in range(self._Gaussian_set_num):
                gamma[:, label] = self._p[label] * multivariate_normal.pdf(data, 
                                                    self._mean[label,:], self._covar[label,:,:],
                                                    allow_singular=True)
            # normalize
            gamma /= np.sum(gamma, axis=1)[:,np.newaxis]

            # M-step
            '''
            theta_t+1 = argmax(Q) 
            pk_t+1 = 1/N * Sum(i=1:N)( p(zi_t|xi,theta_t) )
            meank_t+1 = Sum(i=1:N)(p(zi|xi,theta_t) x xi) / Sum(i=1:N)( p(zi|xi,theta_t) )
            covark_t+1 = Sum(i=1:N)(p(zi|xi,theta_t) x (xi-meank_t+1)(xi-meank_t+1)T) 
                         / Sum(i=1:N)( p(zi|xi,theta_t) )
            '''
            self._p = np.mean(gamma, axis=0)
            self._mean = np.dot(gamma.T, data) / np.sum(gamma, axis=0)[:,np.newaxis]
            for label in range(self._Gaussian_set_num):
                diff_mean = data - self._mean[label, :]
                gamma_diag = np.diag(gamma[:,label])
                self._covar[label,:,:] = np.dot(np.dot(diff_mean.T, gamma_diag), diff_mean) \
                                         / np.sum(gamma, axis=0)[:,np.newaxis][label]

            # Get current loss
            N_dis = np.zeros((data_size, self._Gaussian_set_num))
            for label in range(self._Gaussian_set_num):
                '''
                new_loss[:,label] = multivariate_normal.logpdf(data, 
                                                    self._mean[label, :], self._covar[label, :,:], 
                                                    allow_singular=True)
                '''
                dist = multivariate_normal(self._mean[label], self._covar[label],allow_singular=True)
                N_dis[:,label] = self._p[label] * dist.pdf(label)
            new_loss = np.log(np.mean(N_dis, axis=1)) # point probability
            new_loss = np.sum(new_loss) # sum probability

            if t % 5 == 0:
                print("Iteration: %d Loss: %0.6f" %(t, new_loss))
            if abs(new_loss - self._loss) < 10e-10:
                break
            self._loss = new_loss

        # Store model
        GMM_model = {'p': self._p, 'mean': self._mean, 'covariance':self._covar}
        with open(self._model_file_path, 'wb') as f:
            pickle.dump(GMM_model, f)
        print('GMM model trained')

    def predict_label(self, points):
        # Check dimension
        assert(self._data_dim == points.shape[1])

        # Get the most likely label for each point
        labels = np.zeros((points.shape[0], self._Gaussian_set_num))
        for label in range(self._Gaussian_set_num):
            labels[:,label] = self._p[label] * multivariate_normal.pdf(
                                                points, self._mean[label,:], self._covar[label],
                                                allow_singular=True)
        labels = np.argmax(labels, axis=1)
        return labels 

    def predict_value(self, s, known_indices=np.array([0, 1, 2, 3])):
        # Gaussian Mixed Regression
        '''
        Predict missing value with regression
        t_lat = argmax p(s, t_lat | theta)
        '''
        data_n, existed_dim = s.shape
        missing_dim = self._mean.shape[1] - existed_dim
        missing_indices = np.ones(self._mean.shape[1], dtype=np.bool)
        missing_indices[known_indices] = False
        missing_indices, = np.where(missing_indices)

        Y = np.zeros((data_n, missing_dim))
        
        # For each point
        for n in range(data_n):
            p = np.zeros(self._Gaussian_set_num)
            mean = np.zeros((self._Gaussian_set_num, missing_dim))
            covar = np.zeros((self._Gaussian_set_num, missing_dim, missing_dim))
            
            # For each MVN
            for label in range(self._Gaussian_set_num):
                # Multivariate normal Regression
                # Compute prior
                mean_temp = self._mean[label][known_indices]
                covar_temp = self._covar[label][np.ix_(known_indices, known_indices)]
                try:
                    C = covar_temp
                    L = scipy.linalg.cholesky(C, lower=True)
                except np.linalg.LinAlgError:
                    C = covar_temp + 1e-6 * np.eye(existed_dim)
                    L = scipy.linalg.cholesky(C, lower=True)
                D = np.atleast_2d(s[n]) - mean_temp
                cov_sol = scipy.linalg.solve_triangular(L, D.T, lower=True).T
                norm = 0.5 / np.pi ** (0.5 * existed_dim) / scipy.linalg.det(L)
                
                p[label] = self._p[label] * norm * np.exp(-0.5 * np.sum(cov_sol**2, axis=1))
                
                # Compute mean and covariance
                # split covariance
                cov_12 = self._covar[label][np.ix_(missing_indices, known_indices)]
                cov_11 = self._covar[label][np.ix_(missing_indices, missing_indices)]
                cov_22 = self._covar[label][np.ix_(known_indices, known_indices)]
                prec_22 = np.linalg.pinv(cov_22)
                regression_coeffs = cov_12.dot(prec_22)

                mean[label] = self._mean[label][missing_indices] + regression_coeffs.dot((s[n] - self._mean[label][known_indices]).T).T
                covar[label] = cov_11 - regression_coeffs.dot(cov_12.T)

            # Prior normalization
            p /= p.sum()

            # Predicted mean
            Y[n] = np.dot(p, mean)

        return Y


# Test GMM
if __name__ == "__main__":
    gmm = GMM()
    data = np.array([[0.3, 0.6], [0.4, 0.6], [0.5, 0.6], 
                     [0, 0], [0.1, 0.3], [0.1, 0.2], 
                     [0.6, 0.2], [0.8, 0.1], [0.4, 0.2], [0.9, 0.0]])
    gmm.train(data, Gaussian_set_num=3, iteration_t=40)
    label_value = gmm.predict_label(data)

    x = np.linspace(0, 1, 10)[:, np.newaxis]
    pred = gmm.predict_value(x, np.array([0]))

    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse
    def draw_ellipse(position, covariance, ax=None, **kwargs):
        """Draw an ellipse with a given position and covariance"""
        ax = ax or plt.gca()
        
        # Convert covariance to principal axes
        if covariance.shape == (2, 2):
            U, s, Vt = np.linalg.svd(covariance)
            angle = np.degrees(np.arctan2(U[1, 0], U[0, 0]))
            width, height = 2 * np.sqrt(s)
        else:
            angle = 0
            width, height = 2 * np.sqrt(covariance)
        
        # Draw the Ellipse
        for nsig in range(1, 4):
            ax.add_patch(Ellipse(position, nsig*width, nsig*height, angle, **kwargs))

    plt.figure()
    plt.scatter(data[:, 0], data[:, 1], c=label_value)
    plt.scatter(x, pred, c='r')
    for pos, covar, w in zip(gmm._mean, gmm._covar, gmm._p):
        draw_ellipse(pos, covar, alpha = w)
    plt.show()
