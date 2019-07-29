pic_num = 1;
PSM_q0 = zeros(6,1);
PSM_q = [PSM_q0];
time_delta = 0.01;
lambda = 1/time_delta;
duration = 5;
PSM_Model = PSM_DH_Model();
[x0,J0] = FK_Jacob_Geometry(PSM_q0,PSM_Model.DH, PSM_Model.tip, PSM_Model.method);
global PSM_Model
d_size = size(mtm_x);
for i = 1:d_size(3)-1
    qt = PSM_q(:,end);
    [xt,Jt] = FK_Jacob_Geometry(qt,PSM_Model.DH, PSM_Model.tip, PSM_Model.method);
    xd_t = MTM_to_PSM_Mapping(mtm_x(:,:,i));
    [xe_t, delta_theta] = T_Error(xt,xd_t);
    vd_t = psm_xdot_dsr(:,i);
    qdot_t = Inv_Jacob_Control(xe_t, vd_t, Jt, lambda);
    PSM_q = [PSM_q, qt+qdot_t*time_delta];
    if(mod(i,25) == 0)
        PSM_graphical(qt,xe_t,delta_theta);
        F = getframe(gcf);
        I = frame2im(F);
        [I,map]=rgb2ind(I,256);
        if pic_num == 1
            imwrite(I,map,'test2.gif','gif', 'Loopcount',inf,'DelayTime',0.2);
        else
            imwrite(I,map,'test2.gif','gif','WriteMode','append','DelayTime',0.2);
        end
        pic_num = pic_num + 1;
    end
    if(i == d_size(3)-1)
        i = d_size(3)-1;
    end


end





