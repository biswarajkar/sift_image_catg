% Starter code referred from code by James Hays and Sam Birch for CS 143,
% Brown University

%% Step 0: Set up parameters, vlfeat, category list, and image paths.

% Set up paths to VLFeat functions.
disp(datetime('now'));
% See http://www.vlfeat.org/matlab/matlab.html for VLFeat Matlab documentation
% This should work on 32 and 64 bit versions of Windows, MacOS, and Linux
run('vlfeat-0.9.20/toolbox/vl_setup')
data_path = '../data/';

%This is the list of categories / directories to use. The categories are
%somewhat sorted by similarity (indoor and then urban and then rural).
categories = {'Kitchen', 'Store', 'Bedroom', 'LivingRoom', 'Office', ...
    'Industrial', 'Suburb', 'InsideCity', 'TallBuilding', 'Street', ...
    'Highway', 'OpenCountry', 'Coast', 'Mountain', 'Forest'};

%This list of shortened category names is used later for visualization.
abbr_categories = {'Ki', 'St', 'Bd', 'Lv', 'Of', 'In', 'Sb', ...
    'Ct', 'Bl', 'St', 'Hw', 'Od', 'Cs', 'Mn', 'Fr'};

%Number of training examples per category to use.
num_train_per_cat = 100;

%This function returns cell arrays containing the file path for each train
%and test image, as well as cell arrays with the label of each train and
%test image.
fprintf('Step 0: Getting paths and labels for all training and test data\n')
[train_image_paths, test_image_paths, train_labels, test_labels] = ...
    get_image_paths(data_path, categories, num_train_per_cat);
%   All matrices returend are 1500x1 cells


%% Step 1: Build Visual Word Vocabulary
% The function to construct features should return an N x d matrix, where
% N is the number of paths passed to the function and d is the
% dimensionality of each image representation or the vocabulary size.

%Vocab size is set to 400, corresponding to the 'Strong Features'
%evaluation in Lazebnik's paper
vocab_size=450;

fprintf('Step 1: Build Visual Word Vocabulary\n');
% Calling build_vocabulary.m for buidling vocabulary
if ~exist('vocabulary.mat', 'file')
    fprintf(' - Visual word vocabulary NOT found. Creating a new one from training images.\n')
    vocabulary = build_vocabulary(train_image_paths, vocab_size);
    % This matrix is saved to disk rather than passed in a parameter to
    % avoid recomputing the vocabulary every time at significant expense.
    save('vocabulary.mat', 'vocabulary')
else
    fprintf(' - Visual word vocabulary already exists. Using existing file.\n')
end

%% Step 2: Get bag-of-sifts or use clustered SIFT descriptors to find nearest centroids
% Calling get_bags_of_sifts.m for the bag of SIFTs, on the training images
% and then on the test images. It uses the vocabulary already saved before.

fprintf('Step 2: Get Bag-Of-SIFTs of Images.\n');

if ~exist('train_image_feats.mat', 'file')
    fprintf('   -> Running on images in Training Set\n');
    train_image_feats = get_bags_of_sifts(train_image_paths);
    save('train_image_feats.mat', 'train_image_feats')
else
    fprintf('   -> Loading existing data for Training Set\n');
    load('train_image_feats.mat');
end

if ~exist('test_image_feats.mat', 'file')
    fprintf('   -> Running on images in Test Set\n');
    test_image_feats  = get_bags_of_sifts(test_image_paths);
    save('test_image_feats.mat', 'test_image_feats')
else
    fprintf('   -> Loading existing data for Test Set\n');
    load('test_image_feats.mat');
end

%% Step 3: Classify each test image by training and using the classifier
% The function to classify test features will return an N x 1 cell array,
% where N is the number of test cases and each entry is a string indicating
% the predicted category for each test image. Each entry in
% 'predicted_categories' must be one of the 15 strings in 'categories',
% 'train_labels', and 'test_labels'.

fprintf('Step 3: Classify each test image by training and using the classifier.\n');
predicted_categories = nearest_neighbor_classify(train_image_feats, train_labels, test_image_feats);

%% Step 4: Build a confusion matrix and evaluate the system
% This function will create results_webpage/index.html and various image
% thumbnails each time it is called. View the webpage to help interpret
% the classifier performance.

fprintf('Step 4:  Build a confusion matrix and evaluate the system.\n');
create_results_webpage( train_image_paths,  test_image_paths, ...
    train_labels, test_labels, categories, ...
    abbr_categories, predicted_categories)


%%
disp(datetime('now'));

