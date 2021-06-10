function open_values = sum_values(tree,open_nodes)
    h_weight = 1;
    g_weight = 1;
    open_values = zeros(size(open_nodes));
    for iNode = 1:length(open_nodes)
        open_values(iNode) = sum(tree.Node{open_nodes(iNode)}(:,[tree.idx.g, tree.idx.h]) * [g_weight;h_weight]);
    end
end