function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
% Edges from P1 and P2
triangleAedges = [P1(2,:) - P1(1,:);
                  P1(3,:) - P1(1,:);
                  P1(3,:) - P1(2,:)];
triangleBedges = [P2(2,:) - P2(1,:);
                  P2(3,:) - P2(1,:);
                  P2(3,:) - P2(2,:)];

% Combine edges to create normals
edges = [triangleAedges; triangleBedges];

flag = true; % Başlangıçta kesişiyor gibi varsayalım

% Loop through each edge to check normals
for i = 1:size(edges, 1)
    normal = [-edges(i, 2), edges(i, 1)]; % Normali hesapla
    
    % Projeksiyonları hesaplayın
    proj_P1 = dot(P1, repmat(normal, size(P1, 1), 1), 2) / norm(normal);
    proj_P2 = dot(P2, repmat(normal, size(P2, 1), 1), 2) / norm(normal);

    % Min ve max projeksiyonları bulun
    min_proj_P1 = min(proj_P1);
    max_proj_P1 = max(proj_P1);
    min_proj_P2 = min(proj_P2);
    max_proj_P2 = max(proj_P2);

    % Örtüşmeyi kontrol edin
    if max_proj_P1 < min_proj_P2 || max_proj_P2 < min_proj_P1
        flag = false;
        return;
    end
end
% *******************************************************************
end