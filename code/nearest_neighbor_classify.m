% Starter code referred from code by James Hays and Sam Birch from Brown University

%This function will predict the category for every test image by finding
%the training image with most similar features. Instead of 1 nearest
%neighbor, we will vote based on k nearest neighbors which will increase
%performance.

function predicted_categories = nearest_neighbor_classify(training_image_feats, training_labels, test_image_features)
% image_feats is an N x d matrix, where d is the dimensionality of the
%  feature representation.
% train_labels is an N x 1 cell array, where each entry is a string
%  indicating the ground truth category for each training image.
% test_image_feats is an N x d matrix, where d is the dimensionality of the
%  feature representation.
% predicted_categories is an N x 1 cell array, where each entry is a string
%  indicating the predicted category for each test image.

%Number of nearest neighbors to choose when selecting the best matchces for each image
k = 9;
fprintf('      No. of Nearest Neighbors: %d\n\n',k);

no_of_test_images = size(test_image_features,1);

%Compute feature distances between training and test images
%{
 D = vl_alldist2(X,Y)
    http://www.vlfeat.org/matlab/vl_alldist2.html
    returns the pairwise distance matrix D of the columns of X and Y.
    D(i,j) = sum (X(:,i) - Y(:,j)).^2
  vl_feat represents points as columns vs Matlab represents points as rows.
  So, we will use the transpose.
%}
distances = vl_alldist2(training_image_feats', test_image_features');

%[Y,I] = SORT(X,DIM,MODE) sorts the elements of X in ascending order and
%        returns an index matrix I.
%        Ex: If X = [3 7 5; 0 4 2] then sort(X,1) is [0 4 2; 3 7 5]
%For each test image, we sort the distances from training in ascending order
[~, indices] = sort(distances, 1);

%Get Label/Category Information
all_tr_labels = unique(training_labels);
no_of_tr_labels = size(all_tr_labels, 1);

%Make empty matrix to store each category (row) with the count of test images (column)
%which belong to each corresponding category.
count_of_labels = zeros(no_of_tr_labels, no_of_test_images);

for nth_test_image = 1:no_of_test_images
    %Find the count of nearest k matching categories for each test image
    for nth_label = 1:no_of_tr_labels
        %Take the nearest k labels of each test image
        tr_idx=indices(1:k, nth_test_image);
        top_k_labels = training_labels(tr_idx);
        %matching_indices = strcmp(string, cell_array_of_strings)
        %This will tell which indices in train_labels match a particular category
        count_of_labels(nth_label,nth_test_image) = sum(strcmp(all_tr_labels(nth_label), top_k_labels));
    end
end

% This matrix is saved to disk for reference
save('count_of_labels.mat', 'count_of_labels')

%[Y,I] = MAX(X) returns the indices of the maximum values in vector I.
%      =  MAX(X,[],DIM) operates along the dimension DIM.
%        Ex. If X = [2 8 4; 7 3 9] then max(X,[],1) is [7 8 9]
%We compute the most matched category for each image
[~ , label_indices] = max(count_of_labels,[],1);

%Get the name of the most matched category for each image
predicted_categories = all_tr_labels(label_indices);

end



