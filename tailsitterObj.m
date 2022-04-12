% Author: Boyang Li
% The Hong Kong Polytechnic University
% email: boyang.li@connect.polyu.edu.hk
% Website: https://boyangli.com
% May 2018;

function cost = tailsitterObj2(state,control,p)
sumforce = mean (control(1,:));
variance = var(control(2,:),0,2);
% cost = sumforce + variance;
cost = control(1,:).^2+control(2,:).^2;
end
