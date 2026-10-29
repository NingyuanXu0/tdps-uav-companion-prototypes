%% VL53L0X Measurement Dataset  (同上，保持原始数据不变)
meas = [ ...
  807.0 1200.2 1603.2;
  801.9 1219.8 1605.5;
  808.2 1204.9 1598.1;
  815.2 1196.5 1597.4;
  801.1 1211.6 1613.5;
  801.1 1195.2 1617.8;
  815.6 1206.7 1606.4;
  809.1 1189.3 1615.0;
  799.2 1194.4 1609.9;
  807.3 1206.6 1601.8;
  799.3 1210.9 1609.9;
  799.3 1206.4 1619.3;
  804.9 1204.1 1606.7;
  787.7 1202.6 1619.5;
  789.2 1193.2 1586.0;
  798.5 1199.2 1613.6;
  794.9 1201.3 1607.7;
  805.5 1213.5 1604.6;
  795.7 1207.7 1607.7;
  791.7 1190.9 1591.1;
  814.7 1207.6 1605.2;
  801.2 1201.9 1609.9;
  803.5 1199.6 1618.8;
  791.6 1209.9 1602.9;
  798.6 1213.2 1600.5;
  803.9 1212.5 1603.0;
  793.8 1198.3 1614.3;
  806.0 1202.5 1609.6;
  798.2 1207.7 1602.8;
  800.7 1212.8 1611.1 ];

gt  = [800 1200 1600];
err = meas - gt;     % 30×3 误差矩阵 (mm)

%% Enhanced Visualisation
fig = figure('Position',[100 100 700 450]);

% --- 1. Notched box-whisker with custom width/whisker ---------------
boxH = boxplot(err, 'Labels',{'0.8 m','1.2 m','1.6 m'}, ...
                     'Notch','on', 'Widths',0.55, ...
                     'Whisker',1.5, 'Symbol','');
set(boxH,{'LineWidth'},{1.4});           % 统一线宽

% --- 2. 统一配色并给箱体上色（半透明） -----------------------------
colors = [0.3010 0.7450 0.9330;   % 青
          0.8500 0.3250 0.0980;   % 橙
          0.4660 0.6740 0.1880];  % 绿
boxes  = findobj(gca,'Tag','Box');
for j = 1:length(boxes)
    patch(get(boxes(j),'XData'),get(boxes(j),'YData'), ...
          colors(ceil(j/2),:), ...
          'FaceAlpha',0.28, 'EdgeColor',colors(ceil(j/2),:));
end

% --- 3. 叠加 median diamond 和原始散点 -----------------------------
hold on;
med = median(err);
for k = 1:size(err,2)
    plot(k, med(k), 'kd', 'MarkerFaceColor','k', 'MarkerSize',6);  % diamond
    % jittered scatter
    jitterX = k + 0.12*randn(size(err,1),1);
    scatter(jitterX, err(:,k), 16, colors(k,:), ...
            'filled','MarkerFaceAlpha',0.55, 'MarkerEdgeAlpha',0.55);
end

% --- 4. 美化坐标轴 & 参考线 ----------------------------------------
yline(0,'--k','LineWidth',1);            % 0 误差参考线
ylabel('Measurement error (mm)');
title('VL53L0X Measurement Error at Different Altitudes','FontWeight','bold');
set(gca,'FontName','Arial','FontSize',11, ...
        'GridAlpha',0.3,'GridLineStyle','--');
grid on;  box on;

% --- 5. 高分辨率保存 -----------------------------------------------
exportgraphics(fig,'vl53l0x_error_boxplot_hd.png','Resolution',300);
