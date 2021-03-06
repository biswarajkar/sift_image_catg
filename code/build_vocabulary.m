% Starter code referred from code by James Hays and Sam Birch from Brown University

%This function will sample SIFT descriptors from the training images,
%cluster them with kmeans, and then return the cluster centers.

function vocabulary = build_vocabulary( image_paths, vocab_size )
% The inputs are images,
%           a N x 1 cell array of image paths and
%           the size of the vocabulary
% The output 'vocabulary' is vocab_size x 128. Each row is a cluster
% centroid / visual word.

%Number of key-points to consider for each image
No_of_sample_descriptors = 9;

%Step size for SIFT feature detection
step_size = 10;

fprintf('   Number of key-points to consider for each image: %d\n',No_of_sample_descriptors);
fprintf('   Step Size: %d\n\n',step_size);

%Total Number of images to process
No_of_images = size(image_paths, 1);

%Store all descriptors in a matrix of size 128 x [1500 x 8]
descriptors = zeros(128, No_of_images * No_of_sample_descriptors);

%% Get SIFT features/descriptors for each image
fprintf('   -> Get SIFT descriptors for all images\n');
for counter=1:No_of_images
    % Load each image from the training set
    img = im2single(imread(image_paths{counter}));
    % Get features for each image
    %{
      [locations, SIFT_features] = vl_dsift(img)
      http://www.vlfeat.org/matlab/vl_dsift.html
      'Step',10 -> Extracts a SIFT descriptor each 10 pixels.
      locations is a 2 x n list list of locations, which can be ignored.
      SIFT_features is a 128 x N matrix of SIFT features which we are interested
    %}
    [~, features] = vl_dsift(img, 'Fast', 'Step', step_size);
    tot_no_of_features = size(features,2);
    % We will randomly sample the descriptors from each image instead of taking
    % all descriptors to save memory and speed up the clustering.
    random_features=randi([1 tot_no_of_features],1,No_of_sample_descriptors);
    % So, we take the any of the 'No_of_sample_descriptors' descriptors per 
    % image at random using the randi function to generate 'No_of_sample_descriptors' 
    % random integers from the uniform distribution between 1 and Total number of features.
    descriptors(:, No_of_sample_descriptors * (counter-1) + 1 : No_of_sample_descriptors * counter) ...
                        = features(:,random_features);
end

%% K-Means Clustering/Quantization
%{
Now that we have tens of thousands of SIFT features from many training
images, we will need to cluster them with kmeans/LLOYD algorithm. The
resulting centroids are now our visual word vocabulary.
Matlab has a build in kmeans function, but it is slower, so I use vl_kmeans
[centers, assignments] = vl_kmeans(X, K)
 http://www.vlfeat.org/matlab/vl_kmeans.html
  X is a d x M matrix of sampled SIFT features, where M is the number of
   features sampled.
  K is the number of clusters desired (vocab_size)
  Initialization is the initial data-point selection, we set it to random
   data points (RANDSEL).
  centers is a d x K matrix of cluster centroids. This is the vocabulary we
   need to store.
  assignments is a row vector specifying the assignments of the data X to the
   K centers. We ignore this.
%}
fprintf('   -> Run K-Means Clustering on all descriptors\n');
[centroids, ~] = vl_kmeans(single(descriptors),vocab_size,'Initialization','RANDSEL');
vocabulary = centroids';

