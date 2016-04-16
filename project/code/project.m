% Starter code referred from code by James Hays and Sam Birch for CS 143,
% Brown University

%% Step 1: Set up parameters, vlfeat, category list, and image paths.

%For this project, I report performance for the  Bag of sift features and 
%nearest neighbor classifier

% Set up paths to VLFeat functions.
% See http://www.vlfeat.org/matlab/matlab.html for VLFeat Matlab documentation
% This should work on 32 and 64 bit versions of Windows, MacOS, and Linux
run('vlfeat-0.9.20/toolbox/vl_setup')

% Set the main data path
data_path = '../data/';

%This is the list of categories / directories to use. The categories are
%somewhat sorted by similarity (indoor and then urban and then rural).
categories = {'Kitchen', 'Store', 'Bedroom', 'LivingRoom', 'Office', ...
       'Industrial', 'Suburb', 'InsideCity', 'TallBuilding', 'Street', ...
       'Highway', 'OpenCountry', 'Coast', 'Mountain', 'Forest'};
   
%This list of shortened category names is used later for visualization.
abbr_categories = {'Kit', 'Sto', 'Bed', 'Liv', 'Off', 'Ind', 'Sub', ...
    'Cty', 'Bld', 'St', 'HW', 'OC', 'Cst', 'Mnt', 'For'};

%abbr_categories = {'Ki', 'St', 'Bd', 'Lv', 'Of', 'In', 'Sb', ...
%    'Ct', 'Bl', 'St', 'Hw', 'Od', 'Cs', 'Mn', 'Fr'};
    
%number of training examples per category to use.
num_train_per_cat = 100; 

%This function returns cell arrays containing the file path for each train
%and test image, as well as cell arrays with the label of each train and
%test image.
fprintf('Getting paths and labels for all train and test data\n')
[train_image_paths, test_image_paths, train_labels, test_labels] = ...
    get_image_paths(data_path, categories, num_train_per_cat);
%   train_image_paths  1500x1   cell      
%   test_image_paths   1500x1   cell           
%   train_labels       1500x1   cell         
%   test_labels        1500x1   cell          

%% Step 2: Represent each image with the appropriate feature
% The function to construct features should return an N x d matrix, where
% N is the number of paths passed to the function and d is the 
% dimensionality of each image representation or the vocabulary size.

% Calling build_vocabulary.m for buidling vocabulary
if ~exist('vocabulary.mat', 'file')
    fprintf('Visual word vocabulary NOT found. Creating a new one from training images.\n')
    vocab_size = 500;
    vocabulary = build_vocabulary(train_image_paths, vocab_size);
    % Save a copy of the vocabulary for reference
    save('vocabulary.mat', 'vocabulary')
end

% Calling get_bags_of_sifts.m for the bag of SIFTs
train_image_feats = get_bags_of_sifts(train_image_paths, vocabulary);
test_image_feats  = get_bags_of_sifts(test_image_paths, vocabulary);

%% Step 3: Classify each test image by training and using the classifier
% The function to classify test features will return an N x 1 cell array,
% where N is the number of test cases and each entry is a string indicating
% the predicted category for each test image. Each entry in
% 'predicted_categories' must be one of the 15 strings in 'categories',
% 'train_labels', and 'test_labels'.

predicted_categories = nearest_neighbor_classify(train_image_feats, train_labels, test_image_feats);


%% Step 4: Build a confusion matrix and score the system
% This function will create results_webpage/index.html and various image
% thumbnails each time it is called. View the webpage to help interpret
% the classifier performance. 
create_results_webpage( train_image_paths,  test_image_paths, ...
                        train_labels, test_labels, categories, ...
                        abbr_categories, predicted_categories)

% Interpreting the performance with 100 training examples per category:
%  accuracy  =   0 -> The code is broken.
%  accuracy ~= .07 -> The performance is chance. Something is broken.
%  accuracy ~= .20 - .40 -> Rough performance which might have some
%                           isssues.
%  accuracy ~= .50 -> Rough performance with bag of SIFT and nearest
%                     neighbor classifier. Can reach .60 with K-NN and
%                     different distance metrics.
%  accuracy ~= .60 -> Things are roughly correct with bag of SIFT by a
%                     linear SVM classifier or a different distance metric.
%  accuracy >= .70 -> The parameters have been tuned well. E.g. number
%                     of clusters, SVM regularization, number of patches
%                     sampled when building vocabulary, size and step for
%                     dense SIFT features.
%  accuracy >= .80 -> Spatial information or some additional complementary 
%                     image features has been added in somehow . This
%                     represents state of the art in Lazebnik et al 2006.
%  accuracy >= .85 -> The code has done extremely well. This is the state 
%                     of the art in the 2010 SUN database paper from fusing 
%                     many features. 
%  accuracy >= .90 -> Very good results if classifier was changed to SVM.
%  accuracy >= .96 -> This isn't a realistic number. Some accuracy 
%                     calculation is broken or the classifier is cheating 
%                     and seeing the test labels.




