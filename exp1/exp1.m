%% Options
DISTANCE_TYPE = 1;
%% Create DBs
disp('Extracting features...');
tic
db_gal = get_db('ojos_gal');
db_test = get_db('ojos_test');
toc
disp('[OK]');
%% Compare imgs
d = [10,10];
for n=1:10
    for m=1:10
        d(n,m) = vector_d(db_gal(n,:),db_test(m,:),2);
    end
end
[~,res]=min(d);
res = [1:10;res]';
disp('Results:');
disp('Class  | Result');
disp(res);



