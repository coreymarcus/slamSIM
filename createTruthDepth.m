function depth = createTruthDepth(range, V)
%createTruthDepth converts from a range image to a depth image

%perform multiplication
depth = range.*V(:,:,3);

end