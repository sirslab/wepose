groundTruth = raw_data(:,[18 15:17]);
for i=1:length(groundTruth)
    if (groundTruth(i,1) <0)
        groundTruth(i,:) = - groundTruth(i,:);
    end
end


a = find(groundTruth);
firstViconQindex = a(1);
q_initial = groundTruth(firstViconQindex,:);

figure('Name','GT q');
plot(groundTruth, '--');