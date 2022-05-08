%This program is used for computing  the inverse of the Redundant
%manipulator Newton_Raphson algorithm is employed and the program runs for
%nmax iterations with acceptable margin of error of max_error. user also
%enter the initial value who's tangent is first calculated.

%Initilising the nmax and max_error which will determine the computation
%cost of the program.
nmax=100;
max_error=1e-2;

%Getting the point at which the user needs inverse of. 
disp('ENTER THE POSITION AND ORIENTATION OF THE ROBOT');
x_pos= input('PLEASE ENTER THE VALUE OF x:  ');
y_pos= input('PLEASE ENTER THE VALUE OF y:  ');
alpha= pi/180*input ('PLEASE ENTER THE VALUE OF alpha (in degrees) = '); 
beta = pi/180*input ('PLEASE ENTER THE VALUE OF beta (in degrees)  = '); 
gama = pi/180*input ('PLEASE ENTER THE VALUE OF gama (in degrees)  = ');


ii=1;
error(ii)=10;

%User also inputs the assumption of the solution for all variables
disp('PLEASE ENTER YOUR ASSUMED VALUE FOR theta1, theta2 AND d:');
theta1(ii)=pi/180*input('enter your guess for theta1 in degrees:  ');
theta2(ii)=pi/180*input('enter your guess for theta2 in degrees:  ');
d(ii)=input('enter your guess for d in mm:  ');

%The while loop runs the Newton-Raphson algorithm repeatedly until the
%error values reduces below our set value.
while(ii<nmax)&&(error(ii)>max_error)
    
  mat1=[412.*cos(theta1(ii))+(d(ii)+267).*cos(theta1(ii)+theta2(ii)), (d(ii)+267).*cos(theta1(ii)+theta2(ii)), sin(theta1(ii)+theta2(ii));
      -412.*sin(theta1(ii))-(d(ii)+267).*sin(theta1(ii)+theta2(ii)), -(d(ii)+267).*sin(theta1(ii)+theta2(ii)), cos(theta1(ii)+theta2(ii))];
    mat2=[412.*sin(theta1(ii))+(d(ii)+267).*sin(theta1(ii)+theta2(ii))-x_pos;
        412.*cos(theta1(ii))+(d(ii)+267).*cos(theta1(ii)+theta2(ii))-y_pos];
    theta=[theta1(ii);theta2(ii);d(ii)]-pinv(mat1)*mat2;
    ii=ii+1;
    error(ii)=max(mat2);
end

%Displaying the final values of the desired variables. 
disp('THE SOLUTION OF THE REQUIRED PROBLEM IS:');
disp('The value of theta 1:');
disp(theta(1)*180/pi);
disp('The value of theta1 from dh convention');
disp(-(90-(theta(1)*180/pi)));
disp('The value of theta 2:');
disp(theta(2)*180/pi);
disp('The value of theta2 from dh convention');
disp(-(90-(theta(2)*180/pi)));
disp('The value of displacement d:');
disp(theta(3));

%Using Pieper's solution to split the Inverse solution into two and using
%analytical solution.
R36 = [cos(alpha).*cos(beta),  (cos(alpha).*sin(beta).*sin(gama))- sin(alpha).*cos(gama),  (cos(alpha).*sin(beta).*cos(gama)) + sin(alpha).*sin(gama) ; 
       sin(alpha).*cos(beta),  (sin(alpha).*sin(beta).*sin(gama)) + cos(alpha).*cos(gama), (sin(alpha).*sin(beta).*cos(gama)) - cos(alpha).*sin(gama) ; 
     - sin(beta),               cos(beta).*sin(gama),                                       cos(beta).*sin(gama)] ;

s4=sqrt(1-R36(3,3)^2);
theta4=atan2(s4,R36(3,3));
theta5=atan2(R36(3,2)/(s4),R36(3,1)/(-s4));
theta3=atan2(R36(2,3)/s4,R36(1,3)/s4);
disp('The value of theta 3:');
disp(theta3*180/pi);
disp('The value of theta 4:');
disp(theta4*180/pi);
disp('The value of theta 5:');
disp(theta5*180/pi);