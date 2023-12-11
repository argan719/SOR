

Original author: Hyungtae Lim (shapelim@kaist.ac.kr) <br>
https://github.com/LimHyungTae/pcl_tutorial


### Compile

~/sor/build$ cmake .. <br>
~/sor/build$ make

### 실행

~/sor/build$ ./lec07_sor


SOR로 노이즈 제거 후 <br>
z축 제거해서 2D 이미지로 저장. <br>
Make 파일 만들어서 sor 하나만 CMake로 돌아가도록 만듦.

(lec07_sor 을 갖고와서
SOR (Statistical Outlier Removal) 만 Cmake 로 실행되게 구성,
2d projection 코드를 추가하여
포인트 클라우드에서 노이즈가 제거된 건물의 2차원 도면 생성)

Input: pcd <br>
Output: point cloud visualization, 2d_projection.png



Result <br>
(Before/ After)
<img width="613" alt="image" src="https://github.com/argan719/SOR/assets/64789601/1c30fa74-ef9f-452f-a082-de2318f6d840">
<img width="613" alt="image" src="https://github.com/argan719/SOR/assets/64789601/3372c4f8-9c57-4111-ad80-f33225581e4c">
<img width="612" alt="image" src="https://github.com/argan719/SOR/assets/64789601/7256ebf6-ff7b-4de3-a2c6-13cee7ff2015">
<img width="612" alt="image" src="https://github.com/argan719/SOR/assets/64789601/15a1b076-d305-4d92-8265-1461ca7e0856">

