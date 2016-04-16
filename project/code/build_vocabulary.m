% Starter code referred from code by James Hays and Sam Birch for CS 143,
% Brown University

%This function will sample SIFT descriptors from the training images,
%cluster them with kmeans, and then return the cluster centers.

function vocabulary = build_vocabulary( image_paths, vocab_size )
% The inputs are images, 
%           a N x 1 cell array of image paths and 
%           the size of the vocabulary
% The output 'vocabulary' should be vocab_size x 128. Each row is a cluster
% centroid / visual word.
%{
Functions used:
[locations, SIFT_features] = vl_dsift(img) 
 http://www.vlfeat.org/matlab/vl_dsift.html
 locations is a 2 x n list list of locations, which can be thrown away here
 SIFT_features is a 128 x N matrix of SIFT features

[centers, assignments] = vl_kmeans(X, K)
 http://www.vlfeat.org/matlab/vl_kmeans.html
  X is a d x M matrix of sampled SIFT features, where M is the number of
   features sampled. M should be pretty large!
  K is the number of clusters desired (vocab_size)
  centers is a d x K matrix of cluster centroids. This is the vocabulary.
  We disregard 'assignments'.

  Matlab has a build in kmeans function, but it is slower.
%}

No_of_images = size(image_paths, 1);
N_each = ceil(10000/No_of_images);
descriptors = zeros(128, No_of_images * N_each);

for counter=1:No_of_images
    % Load images from the training set
    img = im2single(imread(image_paths{counter}));
    % Get features for each image
    [~, features] = vl_dsift(img, 'Fast', 'Step', 10);
    % Sample the descriptors from each image
    descriptors(:,N_each * (counter-1) + 1 : N_each * counter) = features(:,1:N_each);
end
    
[centers, ~] = vl_kmeans(descriptors, vocab_size);
vocab = single(centers');

% Load images from the training set. To save computation time, you don't
% necessarily need to sample from all images, although it would be better
% to do so. You can randomly sample the descriptors from each image to save
% memory and speed up the clustering. Or you can simply call vl_dsift with
% a large step size here, but a smaller step size in make_hist.m. 

% For each loaded image, get some SIFT features. You don't have to get as
% many SIFT features as you will in get_bags_of_sift.m, because you're only
% trying to get a representative sample here.

% Once you have tens of thousands of SIFT features from many training
% images, cluster them with kmeans. The resulting centroids are now your
% visual word vocabulary.





