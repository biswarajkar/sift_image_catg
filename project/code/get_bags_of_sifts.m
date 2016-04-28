% Starter code referred from code by James Hays and Sam Birch from Brown University

function image_features = get_bags_of_sifts(image_paths)
% The inputs image_paths is an N x 1 cell array of strings where each string 
% is an image path on the file system.

% This function assumes that 'vocabulary.mat' exists and contains an N x 128
% matrix 'vocabulary' where each row is a kmeans centroid or visual word. This
% matrix is saved to disk rather than passed in a parameter to avoid
% recomputing the vocabulary every time at significant expense.

% image_features is an N x d matrix, where d is the dimensionality of the
% feature representation. In this case, d will equal the number of clusters
% or equivalently the number of entries in each image's histogram.

% We will construct SIFT features here in the same way we did in build_vocabulary.m
% (except for changing the sampling rate) and then assign each local feature
% to its nearest cluster center and build a histogram indicating how many
% times each cluster was used.
% We then normalize the histogram, else a larger image with more SIFT features
% will look very different from a smaller version of the same image.

%Load the Vocabulary created
load('vocabulary.mat')

%Step size for SIFT detection
step_size=5;

fprintf('      Step Size: %d\n\n',step_size);
vocab_size = size(vocabulary, 1);
No_of_images = size(image_paths, 1);
image_features = zeros(No_of_images, vocab_size);

for img_counter=1:No_of_images
    %Print progress on every 300th image processed
    if (mod(img_counter,300)==0)
        fprintf('      ..Processed %d Images\n',img_counter);
    end
    
    img = im2single(imread(image_paths{img_counter}));
    %Extract a SIFT descriptor each step_size pixels
    [~, features] = vl_dsift(img,'Step',step_size);
    features = single(features);
    
    %Find the nearest cluster center in vocabulary for each local feature
    %in the image based on the Euclidean distance
    %[indices, distances] = KNNSEARCH(vocabulary,features) returns a vector
    %   distances containing the distances between each row of features and its
    %   closest point in vocabulary. Each row in 'indices' contains the index of
    %   the nearest neighbor in vocabulary for the corresponding row in features.
    [indices, distances] = knnsearch(vocabulary, features');
    %Get the Histogram count indicating how many times each cluster was used
    imhist=histc(indices, 1:vocab_size);
    %Normalize the histogram, so that a larger image with more SIFT features 
    %will not look very different from a smaller version of the same image.
    imhist_norm=imhist./numel(imhist);
    image_features(img_counter,:) = imhist_norm';
    
end
