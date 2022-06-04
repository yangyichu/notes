let plot_px=20;
let plot_py=20;
let scale_x = 30;
let scale_y = 60;
let plot_w = 4*scale_x;
let plot_int = 20;
let plot_h = 1*scale_y;

let plot00_sx=plot_px+0*plot_w+0*plot_int;let plot00_sy=plot_py;
let plot10_sx=plot_px+1*plot_w+1*plot_int;let plot10_sy=plot_py;
let plot20_sx=plot_px+2*plot_w+2*plot_int;let plot20_sy=plot_py;
let plot30_sx=plot_px+3*plot_w+3*plot_int;let plot30_sy=plot_py;
let plot01_sx=60+plot00_sx;let plot01_sy=plot00_sy+plot_h+30;
let plot11_sx=60+plot10_sx;let plot11_sy=plot10_sy+plot_h+30;
let plot21_sx=60+plot20_sx;let plot21_sy=plot20_sy+plot_h+30;
let plot02_sx=60+plot01_sx;let plot02_sy=plot01_sy+plot_h+30;
let plot12_sx=60+plot11_sx;let plot12_sy=plot11_sy+plot_h+30;
let plot03_sx=60+plot02_sx;let plot03_sy=plot02_sy+plot_h+30;

let plot_ori_x=[[plot00_sx,plot01_sx,plot02_sx,plot03_sx],
[plot10_sx,plot11_sx,plot12_sx],
[plot20_sx,plot21_sx],[plot30_sx]];

let plot_ori_y=[[plot00_sy,plot01_sy,plot02_sy,plot03_sy],
[plot10_sy,plot11_sy,plot12_sy],
[plot20_sy,plot21_sy],[plot30_sy]];


let uu=[0,1,2,3,4];


let resolution=0.02;

let nn=0;
let colors;

function setup() {
    colors=[[color('#f44336'),color('#7b1fa2'),color('#5c6bc0'),color('#448aff')],
    [color('#00695c'),color('#81c784'),color('#d4e157')],
    [color('#ffa000'),color('#ff6e40')],
    [color('#78909c')]];
    createCanvas(600, 400);
    init_canvas();
  }
  
function draw() {
    frameRate(15);
    init_canvas();
    nn=nn+1;
    if(nn==1/resolution){nn=0;}
    u=uu[0]+(uu[uu.length-1]-uu[0])*resolution*nn;
    upi_scatter(u,0,0);
    upi_scatter(u,0,1);
    upi_scatter(u,0,2);
    upi_scatter(u,0,3);
    upi_scatter(u,1,0);
    upi_scatter(u,1,1);
    upi_scatter(u,1,2);
    upi_scatter(u,2,0);
    upi_scatter(u,2,1);
    upi_scatter(u,3,0);
    
}

function plot(x,y,start_x,start_y){
for (i = 0; i < x.length-1; i++) { 
    line(start_x+x[i]*scale_x,start_y-y[i]*scale_y,
        start_x+x[i+1]*scale_x,start_y-y[i+1]*scale_y)
}
}

function init_canvas(){
    background(250);
    for(nrow=0;nrow<4;++nrow){
        for(ncol=0;ncol<4-nrow;++ncol){
            subplot(plot_ori_x[ncol][nrow],plot_ori_y[ncol][nrow], plot_w, plot_h);
            plot_bspline(plot_ori_x[ncol][nrow],plot_ori_y[ncol][nrow],nrow,ncol);
        }
    }
}

function subplot(sx,sy,w,h){
stroke(0);
strokeWeight(0.5);
rect(sx, sy, w, h);//plot0
text('0', sx-5, sy+plot_h+15);
text('1', sx+1*scale_x-5, sy+plot_h+15);
text('2', sx+2*scale_x-5, sy+plot_h+15);
text('3', sx+3*scale_x-5, sy+plot_h+15);
text('4', sx+4*scale_x-5, sy+plot_h+15);
text('1', sx-10, sy);
text('0.5', sx-20, sy+0.5*scale_y);
}

function plot_bspline(sx,sy,p,i){
    stroke(0, 100, 255);
    strokeWeight(4);
    let pt_=[sx+uu[0]*scale_x,sy+plot_h-upi(uu[0],p,i)*scale_y];
    for(n = 0; n < 1/resolution; n++){
        // print("n:"+n);
        let u=uu[0]+(uu[uu.length-1]-uu[0])*resolution*n;
        let y=upi(u,p,i);
        // print([u,y]);
        stroke(colors[p][i]);
        line(pt_[0],pt_[1],sx+u*scale_x,sy+plot_h-y*scale_y);
        pt_=[sx+u*scale_x,sy+plot_h-y*scale_y];
    }
}


function upi(u,p,i){
if(p==0){
    if(u>=uu[i] && u<uu[i+1]){
        return 1.0;
    }
    else{
        return 0.0;
    }
}
else{
    let c1=(u-uu[i])/(uu[i+p]-uu[i]);
    let c2=(uu[i+p+1]-u)/(uu[i+p+1]-uu[i+1]);
    let N1=upi(u,p-1,i);
    let N2=upi(u,p-1,i+1);
    y=c1*N1+c2*N2;
    return y;
}
}

function upi_scatter(u,p,i){
    if(p==0){
            sx=plot_ori_x[i][p];
            sy=plot_ori_y[i][p];
            y=upi(u,p,i);
            stroke(colors[p][i]); // Change the color
            strokeWeight(3);
            line(sx+u*scale_x,sy+plot_h-y*scale_y,sx+u*scale_x,sy+plot_h);
            stroke(colors[p][i]); // Change the color
            strokeWeight(10);
            ellipse(sx+u*scale_x,sy+plot_h-y*scale_y,1,1); 
            
        }
    else{
        let c1=(u-uu[i])/(uu[i+p]-uu[i]);
        let c2=(uu[i+p+1]-u)/(uu[i+p+1]-uu[i+1]);
        let N1=upi(u,p-1,i);
        let N2=upi(u,p-1,i+1);
        y=c1*N1+c2*N2;
        y1=c1*N1;
        y2=c2*N2;
        sx=plot_ori_x[i][p];
        sy=plot_ori_y[i][p];
        // plot__part1
        stroke(colors[p-1][i]); // Change the color
        strokeWeight(8);
        line(sx+u*scale_x,sy+plot_h-y*scale_y,sx+u*scale_x,sy+plot_h-y*scale_y+y1*scale_y);
        // plot__part2
        stroke(colors[p-1][i+1]); // Change the color
        strokeWeight(8);
        line(sx+u*scale_x,sy+plot_h-y2*scale_y,sx+u*scale_x,sy+plot_h);
        
        stroke(colors[p][i]); // Change the color
        strokeWeight(10);
        ellipse(sx+u*scale_x,sy+plot_h-y*scale_y,1,1); 
    }
}
