; Assembly listing for method BepuPhysics.CollisionDetection.DepthRefiner`6[BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder,BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder]:GetNextNormal[BepuPhysics.CollisionDetection.DepthRefiner`6+HasNoNewSupport[BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder,BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder]](byref,byref,byref,byref,byref,byref,byref,byref) (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rsp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 91 single block inlinees; 0 inlinees without PGO data

G_M000_IG01:                ;; offset=0x0000
       push     rdi
       push     rsi
       push     rbp
       push     rbx
       sub      rsp, 0xBF8
       vmovaps  xmmword ptr [rsp+0xBE0], xmm6
       vmovaps  xmmword ptr [rsp+0xBD0], xmm7
       vmovaps  xmmword ptr [rsp+0xBC0], xmm8
       vmovaps  xmmword ptr [rsp+0xBB0], xmm9
       vmovaps  xmmword ptr [rsp+0xBA0], xmm10
       vmovaps  xmmword ptr [rsp+0xB90], xmm11
       vmovaps  xmmword ptr [rsp+0xB80], xmm12
       vmovaps  xmmword ptr [rsp+0xB70], xmm13
       vmovaps  xmmword ptr [rsp+0xB60], xmm14
       vmovaps  xmmword ptr [rsp+0xB50], xmm15
       mov      rax, bword ptr [rsp+0xC40]
       mov      r8, bword ptr [rsp+0xC48]
       mov      rdx, bword ptr [rsp+0xC58]
 
G_M000_IG02:                ;; offset=0x007D
       vmovups  ymm0, ymmword ptr [r8]
       vmovaps  ymm1, ymm0
       vxorps   ymm2, ymm2, ymm2
       vcmpeqps ymm2, ymm2, ymm1
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm3, ymm3, ymm1
       vandps   ymm2, ymm3, ymm2
       vxorps   ymm3, ymm3, ymm3
       vcmpltps ymm3, ymm1, ymm3
       vorps    ymm2, ymm3, ymm2
       vandnps  ymm1, ymm2, ymm1
       vmulps   ymm2, ymm1, ymmword ptr [rax]
       vmovups  ymmword ptr [rsp+0x700], ymm2
       vmulps   ymm3, ymm1, ymmword ptr [rax+0x20]
       vmovups  ymmword ptr [rsp+0x6E0], ymm3
       vmulps   ymm4, ymm1, ymmword ptr [rax+0x40]
       vmovups  ymmword ptr [rsp+0x6C0], ymm4
       vxorps   ymm1, ymm1, ymm1
       vcmpltps ymm1, ymm0, ymm1
       mov      r10, bword ptr [rsp+0xC50]
       vmovups  ymm5, ymmword ptr [r10]
       vsubps   ymm0, ymm5, ymm0
       vblendvps ymm0, ymm5, ymm0, ymm1
       vmulps   ymm0, ymm0, ymm0
       vmovups  ymmword ptr [rsp+0xAE0], ymm0
       lea      r10, bword ptr [rcx+0x60]
       mov      r11, r10
       vmovups  ymm1, ymmword ptr [rcx+0xE0]
       vpor     ymm1, ymm1, ymmword ptr [r9]
       vmovups  ymm5, ymmword ptr [rcx]
       vmovups  ymmword ptr [rcx], ymm5
       vmovups  ymm5, ymmword ptr [rcx+0x20]
       vmovups  ymmword ptr [rcx+0x20], ymm5
       vmovups  ymm5, ymmword ptr [rcx+0x40]
       vmovups  ymmword ptr [rcx+0x40], ymm5
       mov      rbx, r10
       vmovups  ymm5, ymmword ptr [r11]
       vmovups  ymmword ptr [r11], ymm5
       vmovups  ymm5, ymmword ptr [r11+0x20]
       vmovups  ymmword ptr [r11+0x20], ymm5
       vmovups  ymm5, ymmword ptr [r11+0x40]
       vmovups  ymmword ptr [rbx+0x40], ymm5
       vpand    ymm5, ymm1, ymmword ptr [rcx+0xE0]
       vpcmpeqd ymm6, ymm6, ymm6
       vpandn   ymm1, ymm1, ymm6
       vpor     ymm1, ymm1, ymm5
       vmovups  ymmword ptr [rcx+0xE0], ymm1
       lea      r11, bword ptr [rcx+0x100]
       mov      rbx, r11
       mov      rsi, r10
       vmovups  ymm1, ymmword ptr [rbx+0xE0]
       vpor     ymm1, ymm1, ymmword ptr [r9]
       vandps   ymm5, ymm1, ymmword ptr [rbx]
       vandnps  ymm6, ymm1, ymmword ptr [rcx]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rbx], ymm5
       vandps   ymm5, ymm1, ymmword ptr [rbx+0x20]
       vandnps  ymm6, ymm1, ymmword ptr [rcx+0x20]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rbx+0x20], ymm5
 
G_M000_IG03:                ;; offset=0x01AF
       vandps   ymm5, ymm1, ymmword ptr [rbx+0x40]
       vandnps  ymm6, ymm1, ymmword ptr [rcx+0x40]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rbx+0x40], ymm5
       lea      rdi, bword ptr [rbx+0x60]
       lea      rbp, bword ptr [rbx+0x60]
       vandps   ymm5, ymm1, ymmword ptr [rbp]
       vandnps  ymm6, ymm1, ymmword ptr [rsi]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rdi], ymm5
       vandps   ymm5, ymm1, ymmword ptr [rbp+0x20]
       vandnps  ymm6, ymm1, ymmword ptr [rsi+0x20]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rdi+0x20], ymm5
       vandps   ymm5, ymm1, ymmword ptr [rbp+0x40]
       vandnps  ymm6, ymm1, ymmword ptr [rsi+0x40]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rbp+0x40], ymm5
       vpand    ymm5, ymm1, ymmword ptr [rbx+0xE0]
       vpcmpeqd ymm6, ymm6, ymm6
       vpandn   ymm1, ymm1, ymm6
       vpor     ymm1, ymm1, ymm5
       vmovups  ymmword ptr [rbx+0xE0], ymm1
       lea      rbx, bword ptr [rcx+0x200]
       mov      rsi, rbx
       vmovups  ymm1, ymmword ptr [rsi+0xE0]
       vpor     ymm1, ymm1, ymmword ptr [r9]
       vandps   ymm5, ymm1, ymmword ptr [rsi]
       vandnps  ymm6, ymm1, ymmword ptr [rcx]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rsi], ymm5
       vandps   ymm5, ymm1, ymmword ptr [rsi+0x20]
       vandnps  ymm6, ymm1, ymmword ptr [rcx+0x20]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rsi+0x20], ymm5
       vandps   ymm5, ymm1, ymmword ptr [rsi+0x40]
       vandnps  ymm6, ymm1, ymmword ptr [rcx+0x40]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rsi+0x40], ymm5
       lea      rdi, bword ptr [rsi+0x60]
       lea      rbp, bword ptr [rsi+0x60]
       vandps   ymm5, ymm1, ymmword ptr [rbp]
       vandnps  ymm6, ymm1, ymmword ptr [r10]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rdi], ymm5
       vandps   ymm5, ymm1, ymmword ptr [rbp+0x20]
       vandnps  ymm6, ymm1, ymmword ptr [r10+0x20]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rdi+0x20], ymm5
       vandps   ymm5, ymm1, ymmword ptr [rbp+0x40]
       vandnps  ymm6, ymm1, ymmword ptr [r10+0x40]
       vorps    ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rbp+0x40], ymm5
       vpand    ymm5, ymm1, ymmword ptr [rsi+0xE0]
       vpcmpeqd ymm6, ymm6, ymm6
       vpandn   ymm1, ymm1, ymm6
       vpor     ymm1, ymm1, ymm5
       vmovups  ymmword ptr [rsi+0xE0], ymm1
       mov      r10, r11
 
G_M000_IG04:                ;; offset=0x02CB
       vmovups  ymm1, ymmword ptr [r10]
       vmovups  ymm5, ymmword ptr [rcx]
       vsubps   ymm1, ymm1, ymm5
       vmovups  ymm6, ymmword ptr [r10+0x20]
       vmovups  ymmword ptr [rsp+0x80], ymm6
       vmovups  ymm7, ymmword ptr [rcx+0x20]
       vmovups  ymmword ptr [rsp+0x100], ymm7
       vsubps   ymm8, ymm6, ymm7
       vmovups  ymm9, ymmword ptr [r10+0x40]
       vmovups  ymmword ptr [rsp+0x60], ymm9
       vmovups  ymm10, ymmword ptr [rcx+0x40]
       vmovups  ymmword ptr [rsp+0xE0], ymm10
       vsubps   ymm11, ymm9, ymm10
       vmovups  ymm12, ymmword ptr [rbx]
       vmovups  ymmword ptr [rsp+0x20], ymm12
       vsubps   ymm13, ymm5, ymm12
       vmovups  ymm14, ymmword ptr [rbx+0x20]
       vmovups  ymmword ptr [rsp+0xC0], ymm14
       vsubps   ymm15, ymm7, ymm14
       vmovups  ymm0, ymmword ptr [rbx+0x40]
       vmovups  ymmword ptr [rsp+0xA0], ymm0
       vsubps   ymm9, ymm10, ymm0
       mov      r10, r11
       vsubps   ymm6, ymm12, ymmword ptr [r10]
       vmovups  ymmword ptr [rsp+0x6A0], ymm6
       vsubps   ymm14, ymm14, ymmword ptr [rsp+0x80]
       vmovups  ymmword ptr [rsp+0x680], ymm14
       vsubps   ymm0, ymm0, ymmword ptr [rsp+0x60]
       vmovups  ymmword ptr [rsp+0x660], ymm0
       vmulps   ymm12, ymm8, ymm9
       vmulps   ymm4, ymm11, ymm15
       vsubps   ymm4, ymm12, ymm4
       vmulps   ymm12, ymm11, ymm13
       vmulps   ymm10, ymm1, ymm9
       vsubps   ymm10, ymm12, ymm10
       vmulps   ymm12, ymm1, ymm15
       vmulps   ymm3, ymm8, ymm13
       vsubps   ymm3, ymm12, ymm3
       vmulps   ymm12, ymm4, ymm4
       vmulps   ymm7, ymm10, ymm10
       vaddps   ymm7, ymm7, ymm12
       vmulps   ymm12, ymm3, ymm3
       vaddps   ymm7, ymm12, ymm7
       vmovups  ymmword ptr [rsp+0xAC0], ymm7
       vsubps   ymm5, ymm5, ymm2
       vmovups  ymmword ptr [rsp+0x640], ymm5
       vmovups  ymm12, ymmword ptr [rsp+0x100]
       vsubps   ymm12, ymm12, ymmword ptr [rsp+0x6E0]
       vmovups  ymmword ptr [rsp+0x620], ymm12
       vmovups  ymm7, ymmword ptr [rsp+0xE0]
       vsubps   ymm7, ymm7, ymmword ptr [rsp+0x6C0]
       vmovups  ymmword ptr [rsp+0x600], ymm7
       vmovups  ymm2, ymmword ptr [rsp+0x20]
       vsubps   ymm2, ymm2, ymmword ptr [rsp+0x700]
       vmovups  ymmword ptr [rsp+0x5E0], ymm2
       vmovups  ymm0, ymmword ptr [rsp+0xC0]
       vsubps   ymm0, ymm0, ymmword ptr [rsp+0x6E0]
       vmovups  ymmword ptr [rsp+0x5C0], ymm0
       vmovups  ymm14, ymmword ptr [rsp+0xA0]
 
G_M000_IG05:                ;; offset=0x0447
       vsubps   ymm14, ymm14, ymmword ptr [rsp+0x6C0]
       vmovups  ymmword ptr [rsp+0x5A0], ymm14
       vmulps   ymm6, ymm8, ymm7
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmulps   ymm6, ymm11, ymm12
       vmovups  ymmword ptr [rsp+0xB10], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0xB30]
       vsubps   ymm6, ymm6, ymmword ptr [rsp+0xB10]
       vmovups  ymmword ptr [rsp+0x580], ymm6
       vmulps   ymm6, ymm11, ymm5
       vmovups  ymmword ptr [rsp+0xB10], ymm6
       vmulps   ymm6, ymm1, ymm7
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0xB10]
       vsubps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0x560], ymm6
       vmulps   ymm6, ymm1, ymm12
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmulps   ymm6, ymm8, ymm5
       vmovups  ymmword ptr [rsp+0xB10], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0xB30]
       vsubps   ymm6, ymm6, ymmword ptr [rsp+0xB10]
       vmovups  ymmword ptr [rsp+0x540], ymm6
       vmulps   ymm6, ymm15, ymm14
       vmovups  ymmword ptr [rsp+0xB10], ymm6
       vmulps   ymm6, ymm9, ymm0
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0xB10]
       vsubps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0x520], ymm6
       vmulps   ymm6, ymm9, ymm2
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmulps   ymm6, ymm13, ymm14
       vmovups  ymmword ptr [rsp+0xB10], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0xB30]
       vsubps   ymm6, ymm6, ymmword ptr [rsp+0xB10]
       vmovups  ymmword ptr [rsp+0x500], ymm6
       vmulps   ymm6, ymm13, ymm0
       vmovups  ymmword ptr [rsp+0xB10], ymm6
       vmulps   ymm6, ymm15, ymm2
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0xB10]
       vsubps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0x4E0], ymm6
       vmulps   ymm6, ymm4, ymmword ptr [rsp+0x580]
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmulps   ymm6, ymm10, ymmword ptr [rsp+0x560]
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmulps   ymm6, ymm3, ymmword ptr [rsp+0x540]
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xAA0], ymm6
       vmulps   ymm6, ymm4, ymmword ptr [rsp+0x520]
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmulps   ymm6, ymm10, ymmword ptr [rsp+0x500]
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmulps   ymm6, ymm3, ymmword ptr [rsp+0x4E0]
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
 
G_M000_IG06:                ;; offset=0x0622
       vmovups  ymmword ptr [rsp+0xA80], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0xAC0]
       vsubps   ymm6, ymm6, ymmword ptr [rsp+0xA80]
       vsubps   ymm6, ymm6, ymmword ptr [rsp+0xAA0]
       vmovups  ymmword ptr [rsp+0xA60], ymm6
       vxorps   ymm6, ymm6, ymm6
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0xAA0]
       vcmpltps ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xA40], ymm6
       vxorps   ymm6, ymm6, ymm6
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0xA60]
       vcmpltps ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xA20], ymm6
       vxorps   ymm6, ymm6, ymm6
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0xA80]
       vcmpltps ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xA00], ymm6
       vmulps   ymm6, ymm1, ymm1
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmulps   ymm6, ymm8, ymm8
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmulps   ymm6, ymm11, ymm11
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymm14, ymmword ptr [rsp+0x6A0]
       vmulps   ymm14, ymm14, ymm14
       vmovups  ymmword ptr [rsp+0xB30], ymm14
       vmovups  ymm14, ymmword ptr [rsp+0x680]
       vmulps   ymm14, ymm14, ymm14
       vaddps   ymm14, ymm14, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm14
       vmovups  ymm14, ymmword ptr [rsp+0x660]
       vmulps   ymm14, ymm14, ymm14
       vaddps   ymm14, ymm14, ymmword ptr [rsp+0xB30]
       vmulps   ymm0, ymm13, ymm13
       vmovups  ymmword ptr [rsp+0xB30], ymm0
       vmulps   ymm0, ymm15, ymm15
       vaddps   ymm0, ymm0, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm0
       vmulps   ymm0, ymm9, ymm9
       vaddps   ymm0, ymm0, ymmword ptr [rsp+0xB30]
       vcmpeqps ymm2, ymm6, ymm14
       vmovups  ymmword ptr [rsp+0xB30], ymm2
       vxorps   ymm2, ymm2, ymm2
       vpcmpgtd ymm2, ymm2, ymm14
       vandps   ymm2, ymm2, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm2
       vcmpneqps ymm2, ymm6, ymm6
       vorps    ymm2, ymm2, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm2
       vcmpltps ymm2, ymm14, ymm6
       vorps    ymm2, ymm2, ymmword ptr [rsp+0xB30]
       vblendvps ymm2, ymm14, ymm6, ymm2
       vcmpeqps ymm7, ymm2, ymm0
       vmovups  ymmword ptr [rsp+0xB30], ymm7
       vxorps   ymm7, ymm7, ymm7
 
G_M000_IG07:                ;; offset=0x07E4
       vpcmpgtd ymm7, ymm7, ymm0
       vandps   ymm7, ymm7, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm7
       vcmpneqps ymm7, ymm2, ymm2
       vorps    ymm7, ymm7, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm7
       vcmpltps ymm7, ymm0, ymm2
       vorps    ymm7, ymm7, ymmword ptr [rsp+0xB30]
       vblendvps ymm2, ymm0, ymm2, ymm7
       vmovups  ymmword ptr [rsp+0x9E0], ymm2
       vmulps   ymm7, ymm2, ymmword ptr [reloc @RWD00]
       vmovups  ymm2, ymmword ptr [rsp+0xAC0]
       vcmpleps ymm7, ymm2, ymm7
       vmovups  ymmword ptr [rsp+0x9C0], ymm7
       vmovups  ymm2, ymmword ptr [rsp+0x9E0]
       vcmpltps ymm2, ymm2, ymmword ptr [reloc @RWD32]
       vmovups  ymmword ptr [rsp+0x9A0], ymm2
       vpandn   ymm7, ymm2, ymm7
       vmovups  ymmword ptr [rsp+0x980], ymm7
       vmulps   ymm7, ymm4, ymmword ptr [rax]
       vmulps   ymm2, ymm10, ymmword ptr [rax+0x20]
       vaddps   ymm2, ymm2, ymm7
       vmulps   ymm7, ymm3, ymmword ptr [rax+0x40]
       vaddps   ymm2, ymm7, ymm2
       vxorps   ymm7, ymm7, ymm7
       vcmpltps ymm2, ymm2, ymm7
       vbroadcastss ymm7, dword ptr [reloc @RWD64]
       vxorps   ymm12, ymm7, ymm4
       vandps   ymm12, ymm12, ymm2
       vandnps  ymm4, ymm2, ymm4
       vorps    ymm4, ymm4, ymm12
       vxorps   ymm12, ymm7, ymm10
       vandps   ymm12, ymm12, ymm2
       vandnps  ymm10, ymm2, ymm10
       vorps    ymm10, ymm10, ymm12
       vxorps   ymm12, ymm7, ymm3
       vandps   ymm12, ymm12, ymm2
       vandnps  ymm3, ymm2, ymm3
       vorps    ymm3, ymm3, ymm12
       vmovups  ymm2, ymmword ptr [rsp+0xA20]
       vpor     ymm2, ymm2, ymmword ptr [rsp+0xA00]
       vpor     ymm2, ymm2, ymmword ptr [rsp+0xA40]
       vmovups  ymmword ptr [rsp+0x960], ymm2
       vxorps   ymm12, ymm7, ymm5
       vmovups  ymmword ptr [rsp+0x4C0], ymm12
       vxorps   ymm2, ymm7, ymmword ptr [rsp+0x620]
       vmovups  ymmword ptr [rsp+0x4A0], ymm2
       vxorps   ymm2, ymm7, ymmword ptr [rsp+0x600]
       vmovups  ymmword ptr [rsp+0x480], ymm2
       vbroadcastss ymm2, dword ptr [reloc @RWD68]
       vmovups  ymmword ptr [rsp], ymm2
       vmovups  ymmword ptr [rsp+0x940], ymm2
       vmovups  ymm12, ymmword ptr [r9]
       vandps   ymm2, ymm12, ymmword ptr [rcx+0xC0]
       vmovups  ymmword ptr [rsp+0xB30], ymm2
       vbroadcastss ymm2, dword ptr [reloc @RWD72]
       vmovups  ymmword ptr [rsp+0x40], ymm2
       vandnps  ymm12, ymm12, ymm2
       vorps    ymm12, ymm12, ymmword ptr [rsp+0xB30]
 
G_M000_IG08:                ;; offset=0x0970
       vmovups  ymmword ptr [rcx+0xC0], ymm12
       vmovups  ymm12, ymmword ptr [r9]
       vandps   ymm2, ymm12, ymmword ptr [rcx+0x1C0]
       vmovups  ymmword ptr [rsp+0xB30], ymm2
       vxorps   ymm2, ymm2, ymm2
       vandnps  ymm2, ymm12, ymm2
       vorps    ymm2, ymm2, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rcx+0x1C0], ymm2
       vmovups  ymm2, ymmword ptr [r9]
       vandps   ymm12, ymm2, ymmword ptr [rcx+0x2C0]
       vmovups  ymmword ptr [rsp+0xB30], ymm12
       vxorps   ymm12, ymm12, ymm12
       vandnps  ymm2, ymm2, ymm12
       vorps    ymm2, ymm2, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rcx+0x2C0], ymm2
       vmovups  ymm2, ymmword ptr [r9]
       vandps   ymm12, ymm2, ymmword ptr [rcx+0x300]
       vandnps  ymm2, ymm2, ymmword ptr [rsp+0x40]
       vorps    ymm2, ymm2, ymm12
       vmovups  ymmword ptr [rcx+0x300], ymm2
       vmulps   ymm2, ymm5, ymm5
       vmovups  ymm12, ymmword ptr [rsp+0x620]
       vmulps   ymm12, ymm12, ymm12
       vaddps   ymm2, ymm12, ymm2
       vmovups  ymm12, ymmword ptr [rsp+0x600]
       vmulps   ymm12, ymm12, ymm12
       vaddps   ymm2, ymm12, ymm2
       vcmpltps ymm2, ymm2, ymmword ptr [rsp+0xAE0]
       vpand    ymm2, ymm2, ymmword ptr [rsp+0x9A0]
       vpor     ymm2, ymm2, ymmword ptr [r9]
       vmovups  ymmword ptr [r9], ymm2
       vmovups  ymm2, ymmword ptr [rsp+0x960]
       vpor     ymm2, ymm2, ymmword ptr [rsp+0x980]
       vmovups  ymm12, ymmword ptr [r9]
       vpandn   ymm2, ymm12, ymm2
       vxorps   ymm12, ymm12, ymm12
       vpcmpgtd ymm12, ymm12, ymm2
       vptest   ymm12, ymm12
       je       G_M000_IG16
 
G_M000_IG09:                ;; offset=0x0A6C
       vmovups  ymm12, ymmword ptr [rsp+0x40]
       vdivps   ymm12, ymm12, ymm6
       vmovups  ymmword ptr [rsp+0x920], ymm12
       vmovups  ymm12, ymmword ptr [rsp+0x40]
       vdivps   ymm12, ymm12, ymm14
       vmovups  ymmword ptr [rsp+0x900], ymm12
       vmovups  ymm12, ymmword ptr [rsp+0x40]
       vdivps   ymm12, ymm12, ymm0
       vmovups  ymmword ptr [rsp+0x8E0], ymm12
       vmovups  ymm12, ymmword ptr [r11]
       vsubps   ymm12, ymm12, ymmword ptr [rsp+0x700]
       vmovups  ymmword ptr [rsp+0x460], ymm12
       vmovups  ymm12, ymmword ptr [r11+0x20]
       vsubps   ymm12, ymm12, ymmword ptr [rsp+0x6E0]
       vmovups  ymmword ptr [rsp+0x440], ymm12
       vmovups  ymm12, ymmword ptr [r11+0x40]
       vsubps   ymm12, ymm12, ymmword ptr [rsp+0x6C0]
       vmovups  ymmword ptr [rsp+0x420], ymm12
       vmulps   ymm12, ymm1, ymm5
       vmovups  ymmword ptr [rsp+0xB30], ymm12
       vmulps   ymm12, ymm8, ymmword ptr [rsp+0x620]
       vaddps   ymm12, ymm12, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm12
       vmulps   ymm12, ymm11, ymmword ptr [rsp+0x600]
       vaddps   ymm12, ymm12, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0x8C0], ymm12
       vmovups  ymm12, ymmword ptr [rsp+0x6A0]
       vmulps   ymm12, ymm12, ymmword ptr [rsp+0x460]
       vmovups  ymmword ptr [rsp+0xB30], ymm12
       vmovups  ymm12, ymmword ptr [rsp+0x680]
       vmulps   ymm12, ymm12, ymmword ptr [rsp+0x440]
       vaddps   ymm12, ymm12, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm12
       vmovups  ymm12, ymmword ptr [rsp+0x660]
       vmulps   ymm12, ymm12, ymmword ptr [rsp+0x420]
       vaddps   ymm12, ymm12, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0x8A0], ymm12
       vmulps   ymm12, ymm13, ymmword ptr [rsp+0x5E0]
       vmovups  ymmword ptr [rsp+0xB30], ymm12
       vmulps   ymm12, ymm15, ymmword ptr [rsp+0x5C0]
       vaddps   ymm12, ymm12, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm12
       vmulps   ymm12, ymm9, ymmword ptr [rsp+0x5A0]
       vaddps   ymm12, ymm12, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0x880], ymm12
       vxorps   ymm12, ymm7, ymmword ptr [rsp+0x8C0]
       vmovups  ymmword ptr [rsp+0x720], ymm12
       vcmpeqps ymm12, ymm6, ymm12
       vmovups  ymmword ptr [rsp+0xB30], ymm12
       vxorps   ymm12, ymm12, ymm12
       vpcmpgtd ymm12, ymm12, ymm6
       vandps   ymm12, ymm12, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm12
       vcmpneqps ymm12, ymm6, ymm6
       vorps    ymm12, ymm12, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm12
       vcmpltps ymm12, ymm6, ymmword ptr [rsp+0x720]
       vorps    ymm12, ymm12, ymmword ptr [rsp+0xB30]
       vmovups  ymm5, ymmword ptr [rsp+0x720]
 
G_M000_IG10:                ;; offset=0x0C4A
       vblendvps ymm5, ymm5, ymm6, ymm12
       vxorps   ymm6, ymm6, ymm6
       vcmpeqps ymm6, ymm6, ymm5
       vxorps   ymm12, ymm12, ymm12
       vpcmpgtd ymm12, ymm12, ymm5
       vandps   ymm6, ymm12, ymm6
       vxorps   ymm12, ymm12, ymm12
       vcmpltps ymm12, ymm5, ymm12
       vorps    ymm6, ymm12, ymm6
       vandnps  ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rsp+0x860], ymm5
       vxorps   ymm6, ymm7, ymmword ptr [rsp+0x8A0]
       vcmpeqps ymm12, ymm14, ymm6
       vxorps   ymm5, ymm5, ymm5
       vpcmpgtd ymm5, ymm5, ymm14
       vandps   ymm5, ymm5, ymm12
       vcmpneqps ymm12, ymm14, ymm14
       vorps    ymm5, ymm12, ymm5
       vcmpltps ymm12, ymm14, ymm6
       vorps    ymm5, ymm12, ymm5
       vblendvps ymm5, ymm6, ymm14, ymm5
       vxorps   ymm6, ymm6, ymm6
       vcmpeqps ymm6, ymm6, ymm5
       vxorps   ymm12, ymm12, ymm12
       vpcmpgtd ymm12, ymm12, ymm5
       vandps   ymm6, ymm12, ymm6
       vxorps   ymm12, ymm12, ymm12
       vcmpltps ymm12, ymm5, ymm12
       vorps    ymm6, ymm12, ymm6
       vandnps  ymm5, ymm6, ymm5
       vmovups  ymmword ptr [rsp+0x840], ymm5
       vxorps   ymm6, ymm7, ymmword ptr [rsp+0x880]
       vcmpeqps ymm12, ymm0, ymm6
       vxorps   ymm5, ymm5, ymm5
       vpcmpgtd ymm5, ymm5, ymm0
       vandps   ymm5, ymm5, ymm12
       vcmpneqps ymm12, ymm0, ymm0
       vorps    ymm5, ymm12, ymm5
       vcmpltps ymm12, ymm0, ymm6
       vorps    ymm5, ymm12, ymm5
       vblendvps ymm5, ymm6, ymm0, ymm5
       vxorps   ymm6, ymm6, ymm6
       vcmpeqps ymm6, ymm6, ymm5
       vxorps   ymm12, ymm12, ymm12
       vpcmpgtd ymm12, ymm12, ymm5
       vandps   ymm6, ymm12, ymm6
       vxorps   ymm12, ymm12, ymm12
       vcmpltps ymm12, ymm5, ymm12
       vorps    ymm6, ymm12, ymm6
       vandnps  ymm5, ymm6, ymm5
       vmovups  ymm6, ymmword ptr [rsp+0x860]
       vmulps   ymm6, ymm6, ymmword ptr [rsp+0x920]
       vmovups  ymmword ptr [rsp+0x820], ymm6
       vmovups  ymm12, ymmword ptr [rsp+0x840]
       vmulps   ymm12, ymm12, ymmword ptr [rsp+0x900]
       vmovups  ymmword ptr [rsp+0x800], ymm12
       vmulps   ymm5, ymm5, ymmword ptr [rsp+0x8E0]
       vmovups  ymmword ptr [rsp+0x7E0], ymm5
       vmulps   ymm5, ymm1, ymm6
 
G_M000_IG11:                ;; offset=0x0D91
       vmovups  ymmword ptr [rsp+0x400], ymm5
       vmulps   ymm5, ymm8, ymm6
       vmovups  ymmword ptr [rsp+0x3E0], ymm5
       vmulps   ymm5, ymm11, ymm6
       vmovups  ymmword ptr [rsp+0x3C0], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0x6A0]
       vmulps   ymm5, ymm5, ymm12
       vmovups  ymmword ptr [rsp+0x3A0], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0x680]
       vmulps   ymm5, ymm5, ymm12
       vmovups  ymmword ptr [rsp+0x380], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0x660]
       vmulps   ymm5, ymm5, ymm12
       vmovups  ymmword ptr [rsp+0x360], ymm5
       vmulps   ymm5, ymm13, ymmword ptr [rsp+0x7E0]
       vmovups  ymmword ptr [rsp+0x340], ymm5
       vmulps   ymm5, ymm15, ymmword ptr [rsp+0x7E0]
       vmovups  ymmword ptr [rsp+0x320], ymm5
       vmulps   ymm5, ymm9, ymmword ptr [rsp+0x7E0]
       vmovups  ymmword ptr [rsp+0x300], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0x640]
       vaddps   ymm12, ymm5, ymmword ptr [rsp+0x400]
       vmovups  ymmword ptr [rsp+0x2E0], ymm12
       vmovups  ymm12, ymmword ptr [rsp+0x620]
       vaddps   ymm6, ymm12, ymmword ptr [rsp+0x3E0]
       vmovups  ymmword ptr [rsp+0x2C0], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x600]
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0x3C0]
       vmovups  ymmword ptr [rsp+0x2A0], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x460]
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0x3A0]
       vmovups  ymmword ptr [rsp+0x280], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x440]
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0x380]
       vmovups  ymmword ptr [rsp+0x260], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x420]
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0x360]
       vmovups  ymmword ptr [rsp+0x240], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x5E0]
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0x340]
       vmovups  ymmword ptr [rsp+0x220], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x5C0]
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0x320]
       vmovups  ymmword ptr [rsp+0x200], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x5A0]
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0x300]
       vmovups  ymmword ptr [rsp+0x1E0], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x2E0]
       vmulps   ymm6, ymm6, ymm6
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x2C0]
       vmulps   ymm6, ymm6, ymm6
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x2A0]
       vmulps   ymm6, ymm6, ymm6
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0x7C0], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x280]
 
G_M000_IG12:                ;; offset=0x0F7F
       vmulps   ymm6, ymm6, ymm6
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x260]
       vmulps   ymm6, ymm6, ymm6
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x240]
       vmulps   ymm6, ymm6, ymm6
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0x7A0], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x220]
       vmulps   ymm6, ymm6, ymm6
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x200]
       vmulps   ymm6, ymm6, ymm6
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0xB30], ymm6
       vmovups  ymm6, ymmword ptr [rsp+0x1E0]
       vmulps   ymm6, ymm6, ymm6
       vaddps   ymm6, ymm6, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rsp+0x780], ymm6
       vxorps   ymm6, ymm6, ymm6
       vcmpeqps ymm6, ymm6, ymm14
       vxorps   ymm14, ymm14, ymm14
       vcmpeqps ymm0, ymm14, ymm0
       vmovups  ymm14, ymmword ptr [rsp+0x7C0]
       vcmpltps ymm14, ymm14, ymmword ptr [rsp+0x7A0]
       vpor     ymm6, ymm14, ymm6
       vmovups  ymmword ptr [rsp+0x760], ymm6
       vmovups  ymm14, ymmword ptr [rsp+0x7C0]
       vcmpltps ymm14, ymm14, ymmword ptr [rsp+0x780]
       vpor     ymm14, ymm14, ymm0
       vmovups  ymm6, ymmword ptr [rsp+0x7A0]
       vcmpltps ymm6, ymm6, ymmword ptr [rsp+0x780]
       vpor     ymm0, ymm6, ymm0
       vmovups  ymm6, ymmword ptr [rsp+0x760]
       vpand    ymm6, ymm6, ymm14
       vpandn   ymm0, ymm6, ymm0
       vandps   ymm14, ymm6, ymmword ptr [rsp+0x7C0]
       vmovups  ymmword ptr [rsp+0xB30], ymm14
       vandps   ymm14, ymm0, ymmword ptr [rsp+0x7A0]
       vmovups  ymmword ptr [rsp+0xB10], ymm14
       vandnps  ymm14, ymm0, ymmword ptr [rsp+0x780]
       vorps    ymm14, ymm14, ymmword ptr [rsp+0xB10]
       vandnps  ymm14, ymm6, ymm14
       vorps    ymm14, ymm14, ymmword ptr [rsp+0xB30]
       vcmpleps ymm14, ymm14, ymmword ptr [rsp+0xAE0]
       vpand    ymm14, ymm14, ymm2
       vpor     ymm14, ymm14, ymmword ptr [r9]
       vmovups  ymmword ptr [r9], ymm14
       vandps   ymm14, ymm6, ymmword ptr [rsp+0x820]
       vmovups  ymmword ptr [rsp+0xB30], ymm14
       vandps   ymm14, ymm0, ymmword ptr [rsp+0x800]
       vmovups  ymmword ptr [rsp+0xB10], ymm14
       vandnps  ymm14, ymm0, ymmword ptr [rsp+0x7E0]
       vorps    ymm14, ymm14, ymmword ptr [rsp+0xB10]
       vandnps  ymm14, ymm6, ymm14
       vorps    ymm14, ymm14, ymmword ptr [rsp+0xB30]
       vandps   ymm1, ymm1, ymm6
 
G_M000_IG13:                ;; offset=0x1136
       vandnps  ymm13, ymm6, ymm13
       vorps    ymm1, ymm13, ymm1
       vandps   ymm8, ymm8, ymm6
       vandnps  ymm13, ymm6, ymm15
       vorps    ymm8, ymm13, ymm8
       vandps   ymm11, ymm11, ymm6
       vandnps  ymm9, ymm6, ymm9
       vorps    ymm9, ymm9, ymm11
       vandps   ymm11, ymm5, ymm6
       vandnps  ymm13, ymm6, ymmword ptr [rsp+0x5E0]
       vorps    ymm11, ymm13, ymm11
       vmovups  ymmword ptr [rsp+0x1A0], ymm11
       vandps   ymm13, ymm12, ymm6
       vandnps  ymm15, ymm6, ymmword ptr [rsp+0x5C0]
       vorps    ymm13, ymm15, ymm13
       vmovups  ymmword ptr [rsp+0x180], ymm13
       vmovups  ymm15, ymmword ptr [rsp+0x600]
       vandps   ymm13, ymm15, ymm6
       vandnps  ymm11, ymm6, ymmword ptr [rsp+0x5A0]
       vorps    ymm11, ymm11, ymm13
       vandps   ymm13, ymm0, ymmword ptr [rsp+0x6A0]
       vandnps  ymm1, ymm0, ymm1
       vorps    ymm1, ymm1, ymm13
       vandps   ymm13, ymm0, ymmword ptr [rsp+0x680]
       vandnps  ymm8, ymm0, ymm8
       vorps    ymm8, ymm8, ymm13
       vandps   ymm13, ymm0, ymmword ptr [rsp+0x660]
       vandnps  ymm9, ymm0, ymm9
       vorps    ymm9, ymm9, ymm13
       vmovups  ymmword ptr [rsp+0x1C0], ymm9
       vandps   ymm13, ymm0, ymmword ptr [rsp+0x460]
       vandnps  ymm9, ymm0, ymmword ptr [rsp+0x1A0]
       vorps    ymm9, ymm9, ymm13
       vmovups  ymmword ptr [rsp+0x1A0], ymm9
       vandps   ymm13, ymm0, ymmword ptr [rsp+0x440]
       vandnps  ymm9, ymm0, ymmword ptr [rsp+0x180]
       vorps    ymm13, ymm9, ymm13
       vandps   ymm9, ymm0, ymmword ptr [rsp+0x420]
       vandnps  ymm11, ymm0, ymm11
       vorps    ymm11, ymm11, ymm9
       vxorps   ymm7, ymm7, ymm14
       vmulps   ymm1, ymm7, ymm1
       vmulps   ymm8, ymm7, ymm8
       vmulps   ymm7, ymm7, ymmword ptr [rsp+0x1C0]
       vsubps   ymm1, ymm1, ymmword ptr [rsp+0x1A0]
       vmovups  ymmword ptr [rsp+0x160], ymm1
       vsubps   ymm8, ymm8, ymm13
       vmovups  ymmword ptr [rsp+0x140], ymm8
       vsubps   ymm7, ymm7, ymm11
       vmovups  ymmword ptr [rsp+0x120], ymm7
       vxorps   ymm9, ymm9, ymm9
       vcmpeqps ymm9, ymm9, ymm14
       vmovups  ymm11, ymmword ptr [rsp+0x40]
       vcmpeqps ymm13, ymm11, ymm14
       vmovups  ymm7, ymmword ptr [rsp]
       vpand    ymm8, ymm7, ymm9
       vpand    ymm1, ymm13, ymmword ptr [reloc @RWD96]
       vpandn   ymm7, ymm13, ymmword ptr [reloc @RWD128]
       vpor     ymm1, ymm7, ymm1
 
G_M000_IG14:                ;; offset=0x12B1
       vpandn   ymm1, ymm9, ymm1
       vpor     ymm1, ymm1, ymm8
       vpand    ymm1, ymm1, ymm6
       vmovups  ymmword ptr [rsp+0xB30], ymm1
       vpand    ymm7, ymm9, ymmword ptr [reloc @RWD96]
       vpand    ymm8, ymm13, ymmword ptr [reloc @RWD160]
       vpandn   ymm1, ymm13, ymmword ptr [reloc @RWD192]
       vpor     ymm1, ymm1, ymm8
       vpandn   ymm1, ymm9, ymm1
       vpor     ymm1, ymm1, ymm7
       vpand    ymm1, ymm1, ymm0
       vpand    ymm7, ymm9, ymmword ptr [reloc @RWD160]
       vmovups  ymm8, ymmword ptr [rsp]
       vpand    ymm8, ymm8, ymm13
       vpandn   ymm13, ymm13, ymmword ptr [reloc @RWD224]
       vpor     ymm8, ymm13, ymm8
       vpandn   ymm8, ymm9, ymm8
       vpor     ymm7, ymm8, ymm7
       vpandn   ymm7, ymm0, ymm7
       vpor     ymm1, ymm7, ymm1
       vpandn   ymm1, ymm6, ymm1
       vpor     ymm1, ymm1, ymmword ptr [rsp+0xB30]
       vpand    ymm1, ymm1, ymm2
       vpandn   ymm7, ymm2, ymmword ptr [rsp+0x940]
       vpor     ymm1, ymm7, ymm1
       vandps   ymm7, ymm2, ymmword ptr [rsp+0x160]
       vandnps  ymm8, ymm2, ymmword ptr [rsp+0x4C0]
       vorps    ymm7, ymm8, ymm7
       vandps   ymm8, ymm2, ymmword ptr [rsp+0x140]
       vandnps  ymm9, ymm2, ymmword ptr [rsp+0x4A0]
       vorps    ymm8, ymm9, ymm8
       vandps   ymm9, ymm2, ymmword ptr [rsp+0x120]
       vandnps  ymm13, ymm2, ymmword ptr [rsp+0x480]
       vorps    ymm9, ymm13, ymm9
       vsubps   ymm13, ymm11, ymm14
       vmovups  ymmword ptr [rsp+0x740], ymm13
       vandps   ymm13, ymm6, ymm13
       vmovups  ymmword ptr [rsp+0xB30], ymm13
       vxorps   ymm13, ymm13, ymm13
       vandps   ymm13, ymm13, ymm0
       vmovups  ymmword ptr [rsp+0xB10], ymm13
       vandnps  ymm13, ymm0, ymm14
       vorps    ymm13, ymm13, ymmword ptr [rsp+0xB10]
       vandnps  ymm13, ymm6, ymm13
       vorps    ymm13, ymm13, ymmword ptr [rsp+0xB30]
       vandps   ymm13, ymm13, ymm2
       vmovups  ymmword ptr [rsp+0xB30], ymm13
       vandnps  ymm13, ymm2, ymmword ptr [rcx+0xC0]
       vorps    ymm13, ymm13, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rcx+0xC0], ymm13
       vandps   ymm13, ymm6, ymm14
       vmovups  ymmword ptr [rsp+0xB30], ymm13
       vandps   ymm13, ymm0, ymmword ptr [rsp+0x740]
       vmovups  ymmword ptr [rsp+0xB10], ymm13
       vxorps   ymm13, ymm13, ymm13
       vandnps  ymm13, ymm0, ymm13
       vorps    ymm13, ymm13, ymmword ptr [rsp+0xB10]
       vandnps  ymm13, ymm6, ymm13
       vorps    ymm13, ymm13, ymmword ptr [rsp+0xB30]
 
G_M000_IG15:                ;; offset=0x1433
       vandps   ymm13, ymm13, ymm2
       vmovups  ymmword ptr [rsp+0xB30], ymm13
       vandnps  ymm13, ymm2, ymmword ptr [rcx+0x1C0]
       vorps    ymm13, ymm13, ymmword ptr [rsp+0xB30]
       vmovups  ymmword ptr [rcx+0x1C0], ymm13
       vxorps   ymm13, ymm13, ymm13
       vandps   ymm13, ymm13, ymm6
       vandps   ymm14, ymm0, ymm14
       vandnps  ymm0, ymm0, ymmword ptr [rsp+0x740]
       vorps    ymm0, ymm0, ymm14
       vandnps  ymm0, ymm6, ymm0
       vorps    ymm0, ymm0, ymm13
       vandps   ymm0, ymm0, ymm2
       vandnps  ymm2, ymm2, ymmword ptr [rcx+0x2C0]
       vorps    ymm0, ymm2, ymm0
       vmovups  ymmword ptr [rcx+0x2C0], ymm0
       vmovups  ymmword ptr [rsp+0x4C0], ymm7
       vmovups  ymmword ptr [rsp+0x4A0], ymm8
       vmovups  ymmword ptr [rsp+0x480], ymm9
       vmovups  ymmword ptr [rsp+0x940], ymm1
 
G_M000_IG16:                ;; offset=0x14BA
       vpcmpeqd ymm0, ymm0, ymm0
       vpxor    ymm0, ymm0, ymmword ptr [rsp+0x9C0]
       vmovups  ymm2, ymmword ptr [rsp+0x960]
       vpandn   ymm0, ymm2, ymm0
       vmovups  ymm2, ymmword ptr [r9]
       vpandn   ymm0, ymm2, ymm0
       vxorps   ymm2, ymm2, ymm2
       vpcmpgtd ymm2, ymm2, ymm0
       vptest   ymm2, ymm2
       je       G_M000_IG18
 
G_M000_IG17:                ;; offset=0x14F0
       vmulps   ymm2, ymm4, ymm5
       vmulps   ymm5, ymm10, ymmword ptr [rsp+0x620]
       vaddps   ymm2, ymm5, ymm2
       vmulps   ymm5, ymm3, ymmword ptr [rsp+0x600]
       vaddps   ymm2, ymm5, ymm2
       vmulps   ymm2, ymm2, ymm2
       vmovups  ymm6, ymmword ptr [rsp+0xAC0]
       vmulps   ymm5, ymm6, ymmword ptr [rsp+0xAE0]
       vcmpltps ymm2, ymm2, ymm5
       vpand    ymm2, ymm2, ymm0
       vpor     ymm2, ymm2, ymmword ptr [r9]
       vmovups  ymmword ptr [r9], ymm2
       vandps   ymm2, ymm4, ymm0
       vandnps  ymm4, ymm0, ymmword ptr [rsp+0x4C0]
       vorps    ymm7, ymm4, ymm2
       vandps   ymm2, ymm10, ymm0
       vandnps  ymm4, ymm0, ymmword ptr [rsp+0x4A0]
       vorps    ymm8, ymm4, ymm2
       vandps   ymm2, ymm3, ymm0
       vandnps  ymm3, ymm0, ymmword ptr [rsp+0x480]
       vorps    ymm9, ymm3, ymm2
       vpand    ymm2, ymm0, ymmword ptr [reloc @RWD256]
       vpandn   ymm1, ymm0, ymmword ptr [rsp+0x940]
       vpor     ymm1, ymm1, ymm2
       vandps   ymm2, ymm0, ymmword ptr [rsp+0xA60]
       vandnps  ymm3, ymm0, ymmword ptr [rcx+0xC0]
       vorps    ymm2, ymm3, ymm2
       vmovups  ymmword ptr [rcx+0xC0], ymm2
       vandps   ymm2, ymm0, ymmword ptr [rsp+0xA80]
       vandnps  ymm3, ymm0, ymmword ptr [rcx+0x1C0]
       vorps    ymm2, ymm3, ymm2
       vmovups  ymmword ptr [rcx+0x1C0], ymm2
       vandps   ymm2, ymm0, ymmword ptr [rsp+0xAA0]
       vandnps  ymm3, ymm0, ymmword ptr [rcx+0x2C0]
       vorps    ymm2, ymm3, ymm2
       vmovups  ymmword ptr [rcx+0x2C0], ymm2
       vandps   ymm2, ymm0, ymm6
       vandnps  ymm3, ymm0, ymmword ptr [rcx+0x300]
       vorps    ymm2, ymm3, ymm2
       vmovups  ymmword ptr [rcx+0x300], ymm2
       vmovups  ymmword ptr [rsp+0x4C0], ymm7
       vmovups  ymmword ptr [rsp+0x4A0], ymm8
       vmovups  ymmword ptr [rsp+0x480], ymm9
       vmovups  ymmword ptr [rsp+0x940], ymm1
 
G_M000_IG18:                ;; offset=0x1612
       vmovups  ymm2, ymmword ptr [rsp]
       vmovups  ymm1, ymmword ptr [rsp+0x940]
       vpand    ymm2, ymm2, ymm1
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm2, ymm2, ymm3
       vmovups  ymmword ptr [rcx+0xE0], ymm2
       vpand    ymm2, ymm1, ymmword ptr [reloc @RWD96]
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm2, ymm2, ymm3
       vmovups  ymmword ptr [rcx+0x1E0], ymm2
       vpand    ymm1, ymm1, ymmword ptr [reloc @RWD160]
       vxorps   ymm2, ymm2, ymm2
       vpcmpgtd ymm1, ymm1, ymm2
       vmovups  ymmword ptr [rcx+0x2E0], ymm1
       vxorps   ymm1, ymm1, ymm1
       vpcmpeqd ymm1, ymm1, ymmword ptr [r9]
       vptest   ymm1, ymm1
       je       G_M000_IG20
 
G_M000_IG19:                ;; offset=0x1678
       vbroadcastss ymm1, dword ptr [reloc @RWD288]
       vmovups  ymm7, ymmword ptr [rsp+0x4C0]
       vmulps   ymm2, ymm1, ymm7
       vmovups  ymm8, ymmword ptr [rsp+0x4A0]
       vmulps   ymm3, ymm1, ymm8
       vmovups  ymm9, ymmword ptr [rsp+0x480]
       vmulps   ymm1, ymm1, ymm9
       vmovups  ymm4, ymmword ptr [rsp+0x700]
       vaddps   ymm2, ymm4, ymm2
       vmovups  ymm4, ymmword ptr [rsp+0x6E0]
       vaddps   ymm3, ymm4, ymm3
       vmovups  ymm4, ymmword ptr [rsp+0x6C0]
       vaddps   ymm1, ymm4, ymm1
       vmovups  ymm4, ymmword ptr [r8]
       vxorps   ymm5, ymm5, ymm5
       vcmpleps ymm4, ymm4, ymm5
       vpor     ymm0, ymm4, ymm0
       vandps   ymm4, ymm0, ymm7
       vandnps  ymm2, ymm0, ymm2
       vorps    ymm7, ymm2, ymm4
       vandps   ymm2, ymm0, ymm8
       vandnps  ymm3, ymm0, ymm3
       vorps    ymm8, ymm3, ymm2
       vandps   ymm2, ymm0, ymm9
       vandnps  ymm0, ymm0, ymm1
       vorps    ymm9, ymm0, ymm2
       vmulps   ymm0, ymm7, ymm7
       vmulps   ymm1, ymm8, ymm8
       vaddps   ymm0, ymm1, ymm0
       vmulps   ymm1, ymm9, ymm9
       vaddps   ymm0, ymm1, ymm0
       vsqrtps  ymm0, ymm0
       vmovups  ymm11, ymmword ptr [rsp+0x40]
       vdivps   ymm1, ymm11, ymm0
       vmulps   ymm0, ymm1, ymm7
       vmovups  ymmword ptr [rdx], ymm0
       vmulps   ymm0, ymm1, ymm8
       vmovups  ymmword ptr [rdx+0x20], ymm0
       vmulps   ymm0, ymm1, ymm9
       vmovups  ymmword ptr [rdx+0x40], ymm0
 
G_M000_IG20:                ;; offset=0x1749
       vzeroupper 
       vmovaps  xmm6, xmmword ptr [rsp+0xBE0]
       vmovaps  xmm7, xmmword ptr [rsp+0xBD0]
       vmovaps  xmm8, xmmword ptr [rsp+0xBC0]
       vmovaps  xmm9, xmmword ptr [rsp+0xBB0]
       vmovaps  xmm10, xmmword ptr [rsp+0xBA0]
       vmovaps  xmm11, xmmword ptr [rsp+0xB90]
       vmovaps  xmm12, xmmword ptr [rsp+0xB80]
       vmovaps  xmm13, xmmword ptr [rsp+0xB70]
       vmovaps  xmm14, xmmword ptr [rsp+0xB60]
       vmovaps  xmm15, xmmword ptr [rsp+0xB50]
       add      rsp, 0xBF8
       pop      rbx
       pop      rbp
       pop      rsi
       pop      rdi
       ret      
 
RWD00  	dq	2EDBE6FF2EDBE6FFh, 2EDBE6FF2EDBE6FFh, 2EDBE6FF2EDBE6FFh, 2EDBE6FF2EDBE6FFh
RWD32  	dq	283424DC283424DCh, 283424DC283424DCh, 283424DC283424DCh, 283424DC283424DCh
RWD64  	dd	80000000h		;        -0
RWD68  	dd	00000001h		; 1.4013e-45
RWD72  	dd	3F800000h		;         1
RWD76  	dd	00000000h, 00000000h, 00000000h, 00000000h, 00000000h
RWD96  	dq	0000000200000002h, 0000000200000002h, 0000000200000002h, 0000000200000002h
RWD128 	dq	0000000300000003h, 0000000300000003h, 0000000300000003h, 0000000300000003h
RWD160 	dq	0000000400000004h, 0000000400000004h, 0000000400000004h, 0000000400000004h
RWD192 	dq	0000000600000006h, 0000000600000006h, 0000000600000006h, 0000000600000006h
RWD224 	dq	0000000500000005h, 0000000500000005h, 0000000500000005h, 0000000500000005h
RWD256 	dq	0000000700000007h, 0000000700000007h, 0000000700000007h, 0000000700000007h
RWD288 	dd	40800000h		;         4

; Total bytes of code 6066

; Assembly listing for method BepuPhysics.CollisionDetection.DepthRefiner`6[BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder,BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder]:GetNextNormal[BepuPhysics.CollisionDetection.DepthRefiner`6+HasNewSupport[BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder,BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder]](byref,byref,byref,byref,byref,byref,byref,byref) (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rsp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 126 single block inlinees; 0 inlinees without PGO data

G_M000_IG01:                ;; offset=0x0000
       push     r15
       push     r14
       push     r13
       push     r12
       push     rdi
       push     rsi
       push     rbp
       push     rbx
       sub      rsp, 0xD58
       vmovaps  xmmword ptr [rsp+0xD40], xmm6
       vmovaps  xmmword ptr [rsp+0xD30], xmm7
       vmovaps  xmmword ptr [rsp+0xD20], xmm8
       vmovaps  xmmword ptr [rsp+0xD10], xmm9
       vmovaps  xmmword ptr [rsp+0xD00], xmm10
       vmovaps  xmmword ptr [rsp+0xCF0], xmm11
       vmovaps  xmmword ptr [rsp+0xCE0], xmm12
       vmovaps  xmmword ptr [rsp+0xCD0], xmm13
       vmovaps  xmmword ptr [rsp+0xCC0], xmm14
       vmovaps  xmmword ptr [rsp+0xCB0], xmm15
       mov      rax, bword ptr [rsp+0xDC0]
       mov      r11, bword ptr [rsp+0xDC8]
       mov      r10, bword ptr [rsp+0xDD8]
 
G_M000_IG02:                ;; offset=0x0085
       vmovups  ymm0, ymmword ptr [r11]
       vmovaps  ymm1, ymm0
       vxorps   ymm2, ymm2, ymm2
       vcmpeqps ymm2, ymm2, ymm1
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm3, ymm3, ymm1
       vandps   ymm2, ymm3, ymm2
       vxorps   ymm3, ymm3, ymm3
       vcmpltps ymm3, ymm1, ymm3
       vorps    ymm2, ymm2, ymm3
       vxorps   ymm4, ymm4, ymm4
       vandps   ymm4, ymm4, ymm2
       vandnps  ymm1, ymm2, ymm1
       vorps    ymm1, ymm1, ymm4
       vmulps   ymm2, ymm1, ymmword ptr [rax]
       vmovups  ymmword ptr [rsp+0x860], ymm2
       vmulps   ymm4, ymm1, ymmword ptr [rax+0x20]
       vmovups  ymmword ptr [rsp+0x840], ymm4
       vmulps   ymm5, ymm1, ymmword ptr [rax+0x40]
       vmovups  ymmword ptr [rsp+0x820], ymm5
       mov      rbx, bword ptr [rsp+0xDD0]
       vmovups  ymm1, ymmword ptr [rbx]
       vsubps   ymm0, ymm1, ymm0
       vandps   ymm0, ymm0, ymm3
       vandnps  ymm1, ymm3, ymm1
       vorps    ymm0, ymm1, ymm0
       vmulps   ymm0, ymm0, ymm0
       vmovups  ymmword ptr [rsp+0xC40], ymm0
       vmovups  ymm1, ymmword ptr [rcx+0xE0]
       vmovups  ymm3, ymmword ptr [rcx+0x1E0]
       vpand    ymm3, ymm3, ymmword ptr [rcx+0x2E0]
       vpand    ymm3, ymm3, ymm1
       vmovups  ymm6, ymmword ptr [r9]
       vpandn   ymm3, ymm6, ymm3
       vpor     ymm1, ymm6, ymm1
       vandps   ymm6, ymm1, ymmword ptr [rcx]
       vandnps  ymm7, ymm1, ymmword ptr [rdx]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [rcx], ymm6
       vandps   ymm6, ymm1, ymmword ptr [rcx+0x20]
       vandnps  ymm7, ymm1, ymmword ptr [rdx+0x20]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [rcx+0x20], ymm6
       vandps   ymm6, ymm1, ymmword ptr [rcx+0x40]
       vandnps  ymm7, ymm1, ymmword ptr [rdx+0x40]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [rcx+0x40], ymm6
       lea      rbx, bword ptr [rcx+0x60]
       mov      rsi, rbx
       mov      rdi, rbx
       vandps   ymm6, ymm1, ymmword ptr [rdi]
       vandnps  ymm7, ymm1, ymmword ptr [r8]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [rsi], ymm6
       vandps   ymm6, ymm1, ymmword ptr [rdi+0x20]
       vandnps  ymm7, ymm1, ymmword ptr [r8+0x20]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [rsi+0x20], ymm6
       vandps   ymm6, ymm1, ymmword ptr [rdi+0x40]
       vandnps  ymm7, ymm1, ymmword ptr [r8+0x40]
 
G_M000_IG03:                ;; offset=0x01AB
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [rdi+0x40], ymm6
       vpand    ymm6, ymm1, ymmword ptr [rcx+0xE0]
       vpcmpeqd ymm7, ymm7, ymm7
       vpandn   ymm1, ymm1, ymm7
       vpor     ymm1, ymm1, ymm6
       vmovups  ymmword ptr [rcx+0xE0], ymm1
       lea      rsi, bword ptr [rcx+0x100]
       mov      rdi, rsi
       vmovups  ymm1, ymmword ptr [rdi+0xE0]
       vpor     ymm1, ymm1, ymmword ptr [r9]
       vandps   ymm6, ymm1, ymmword ptr [rdi]
       vandnps  ymm7, ymm1, ymmword ptr [rdx]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [rdi], ymm6
       vandps   ymm6, ymm1, ymmword ptr [rdi+0x20]
       vandnps  ymm7, ymm1, ymmword ptr [rdx+0x20]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [rdi+0x20], ymm6
       vandps   ymm6, ymm1, ymmword ptr [rdi+0x40]
       vandnps  ymm7, ymm1, ymmword ptr [rdx+0x40]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [rdi+0x40], ymm6
       lea      rbp, bword ptr [rdi+0x60]
       mov      r14, rbp
       mov      r15, rbp
       vandps   ymm6, ymm1, ymmword ptr [r15]
       vandnps  ymm7, ymm1, ymmword ptr [r8]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [r14], ymm6
       vandps   ymm6, ymm1, ymmword ptr [r15+0x20]
       vandnps  ymm7, ymm1, ymmword ptr [r8+0x20]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [r14+0x20], ymm6
       vandps   ymm6, ymm1, ymmword ptr [r15+0x40]
       vandnps  ymm7, ymm1, ymmword ptr [r8+0x40]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [r15+0x40], ymm6
       vpand    ymm6, ymm1, ymmword ptr [rdi+0xE0]
       vpcmpeqd ymm7, ymm7, ymm7
       vpandn   ymm1, ymm1, ymm7
       vpor     ymm1, ymm1, ymm6
       vmovups  ymmword ptr [rdi+0xE0], ymm1
       lea      rdi, bword ptr [rcx+0x200]
       mov      r14, rdi
       vmovups  ymm1, ymmword ptr [r14+0xE0]
       vpor     ymm1, ymm1, ymmword ptr [r9]
       vandps   ymm6, ymm1, ymmword ptr [r14]
       vandnps  ymm7, ymm1, ymmword ptr [rdx]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [r14], ymm6
       vandps   ymm6, ymm1, ymmword ptr [r14+0x20]
       vandnps  ymm7, ymm1, ymmword ptr [rdx+0x20]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [r14+0x20], ymm6
       vandps   ymm6, ymm1, ymmword ptr [r14+0x40]
       vandnps  ymm7, ymm1, ymmword ptr [rdx+0x40]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [r14+0x40], ymm6
       lea      r15, bword ptr [r14+0x60]
 
G_M000_IG04:                ;; offset=0x02DA
       mov      r13, r15
       mov      r12, r15
       vandps   ymm6, ymm1, ymmword ptr [r12]
       vandnps  ymm7, ymm1, ymmword ptr [r8]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [r13], ymm6
       vandps   ymm6, ymm1, ymmword ptr [r12+0x20]
       vandnps  ymm7, ymm1, ymmword ptr [r8+0x20]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [r13+0x20], ymm6
       vandps   ymm6, ymm1, ymmword ptr [r12+0x40]
       vandnps  ymm7, ymm1, ymmword ptr [r8+0x40]
       vorps    ymm6, ymm7, ymm6
       vmovups  ymmword ptr [r12+0x40], ymm6
       vpand    ymm6, ymm1, ymmword ptr [r14+0xE0]
       vpcmpeqd ymm7, ymm7, ymm7
       vpandn   ymm1, ymm1, ymm7
       vpor     ymm1, ymm1, ymm6
       vmovups  ymmword ptr [r14+0xE0], ymm1
       vxorps   ymm1, ymm1, ymm1
       vpcmpgtd ymm1, ymm1, ymm3
       vptest   ymm1, ymm1
       je       G_M000_IG09
 
G_M000_IG05:                ;; offset=0x0355
       mov      r14, rsi
       vmovups  ymm1, ymmword ptr [r14]
       vmovups  ymm6, ymmword ptr [rcx]
       vsubps   ymm1, ymm1, ymm6
       vmovups  ymmword ptr [rsp+0x5C0], ymm1
       vmovups  ymm7, ymmword ptr [r14+0x20]
       vmovups  ymm8, ymmword ptr [rcx+0x20]
       vsubps   ymm9, ymm7, ymm8
       vmovups  ymm10, ymmword ptr [r14+0x40]
       vmovups  ymm11, ymmword ptr [rcx+0x40]
       vsubps   ymm12, ymm10, ymm11
       vmovups  ymmword ptr [rsp+0x5A0], ymm12
       mov      r14, rdi
       vsubps   ymm13, ymm6, ymmword ptr [r14]
       vmovups  ymmword ptr [rsp+0x580], ymm13
       vmovups  ymm14, ymmword ptr [r14+0x20]
       vsubps   ymm15, ymm8, ymm14
       vmovups  ymm1, ymmword ptr [r14+0x40]
       vsubps   ymm13, ymm11, ymm1
       vmovups  ymm12, ymmword ptr [rdx]
       vsubps   ymm6, ymm12, ymm6
       vmovups  ymmword ptr [rsp+0x560], ymm6
       vmovups  ymm6, ymmword ptr [rdx+0x20]
       vsubps   ymm8, ymm6, ymm8
       vmovups  ymmword ptr [rsp+0x540], ymm8
       vmovups  ymm8, ymmword ptr [rdx+0x40]
       vsubps   ymm11, ymm8, ymm11
       vmovups  ymmword ptr [rsp+0x520], ymm11
       mov      r14, rsi
       vsubps   ymm11, ymm12, ymmword ptr [r14]
       vsubps   ymm7, ymm6, ymm7
       vsubps   ymm10, ymm8, ymm10
       vmovups  ymmword ptr [rsp+0x500], ymm10
       mov      r14, rdi
       vsubps   ymm10, ymm12, ymmword ptr [r14]
       vmovups  ymmword ptr [rsp+0x4E0], ymm10
       vsubps   ymm14, ymm6, ymm14
       vmovups  ymmword ptr [rsp+0x4C0], ymm14
       vsubps   ymm1, ymm8, ymm1
       vmovups  ymmword ptr [rsp+0x4A0], ymm1
       vmulps   ymm1, ymm9, ymm13
       vmovups  ymm14, ymmword ptr [rsp+0x5A0]
       vmulps   ymm14, ymm14, ymm15
       vsubps   ymm1, ymm1, ymm14
       vmovups  ymm14, ymmword ptr [rsp+0x5A0]
       vmulps   ymm14, ymm14, ymmword ptr [rsp+0x580]
       vmovups  ymm10, ymmword ptr [rsp+0x5C0]
       vmulps   ymm13, ymm10, ymm13
       vsubps   ymm13, ymm14, ymm13
       vmulps   ymm10, ymm10, ymm15
       vmulps   ymm9, ymm9, ymmword ptr [rsp+0x580]
       vsubps   ymm9, ymm10, ymm9
       vsubps   ymm10, ymm12, ymm2
       vsubps   ymm6, ymm6, ymm4
       vsubps   ymm8, ymm8, ymm5
       vmulps   ymm12, ymm13, ymm8
       vmulps   ymm14, ymm9, ymm6
       vsubps   ymm12, ymm12, ymm14
       vmulps   ymm9, ymm9, ymm10
       vmulps   ymm8, ymm1, ymm8
 
G_M000_IG06:                ;; offset=0x04AE
       vsubps   ymm8, ymm9, ymm8
       vmulps   ymm1, ymm1, ymm6
       vmulps   ymm6, ymm13, ymm10
       vsubps   ymm1, ymm1, ymm6
       vmulps   ymm6, ymm12, ymmword ptr [rsp+0x560]
       vmulps   ymm9, ymm8, ymmword ptr [rsp+0x540]
       vaddps   ymm6, ymm9, ymm6
       vmulps   ymm9, ymm1, ymmword ptr [rsp+0x520]
       vaddps   ymm6, ymm9, ymm6
       vmulps   ymm9, ymm12, ymm11
       vmulps   ymm7, ymm8, ymm7
       vaddps   ymm7, ymm7, ymm9
       vmulps   ymm9, ymm1, ymmword ptr [rsp+0x500]
       vaddps   ymm7, ymm9, ymm7
       vmulps   ymm9, ymm12, ymmword ptr [rsp+0x4E0]
       vmulps   ymm8, ymm8, ymmword ptr [rsp+0x4C0]
       vaddps   ymm8, ymm8, ymm9
       vmulps   ymm1, ymm1, ymmword ptr [rsp+0x4A0]
       vaddps   ymm1, ymm1, ymm8
       vxorps   ymm8, ymm8, ymm8
       vcmpgeps ymm8, ymm6, ymm8
       vxorps   ymm9, ymm9, ymm9
       vcmpltps ymm9, ymm7, ymm9
       vpand    ymm8, ymm9, ymm8
       vxorps   ymm9, ymm9, ymm9
       vcmpgeps ymm7, ymm7, ymm9
       vxorps   ymm9, ymm9, ymm9
       vcmpltps ymm9, ymm1, ymm9
       vpand    ymm7, ymm9, ymm7
       vxorps   ymm9, ymm9, ymm9
       vcmpgeps ymm1, ymm1, ymm9
       vxorps   ymm9, ymm9, ymm9
       vcmpltps ymm6, ymm6, ymm9
       vpand    ymm1, ymm6, ymm1
       vpor     ymm6, ymm8, ymm7
       vpor     ymm6, ymm6, ymm1
       vpcmpeqd ymm9, ymm9, ymm9
       vpxor    ymm6, ymm9, ymm6
       vpcmpeqd ymm9, ymm9, ymm9
       vpand    ymm9, ymm9, ymm6
       vpandn   ymm8, ymm6, ymm8
       vpor     ymm8, ymm8, ymm9
       vpand    ymm6, ymm3, ymm7
       vpor     ymm7, ymm6, ymmword ptr [rcx+0xE0]
       vmovups  ymmword ptr [rcx+0xE0], ymm7
       vandps   ymm7, ymm6, ymmword ptr [rdx]
       vandnps  ymm9, ymm6, ymmword ptr [rcx]
       vorps    ymm7, ymm9, ymm7
       vmovups  ymmword ptr [rcx], ymm7
       vandps   ymm7, ymm6, ymmword ptr [rdx+0x20]
       vandnps  ymm9, ymm6, ymmword ptr [rcx+0x20]
       vorps    ymm7, ymm9, ymm7
       vmovups  ymmword ptr [rcx+0x20], ymm7
       vandps   ymm7, ymm6, ymmword ptr [rdx+0x40]
       vandnps  ymm9, ymm6, ymmword ptr [rcx+0x40]
       vorps    ymm7, ymm9, ymm7
       vmovups  ymmword ptr [rcx+0x40], ymm7
       mov      r14, rbx
       vandps   ymm7, ymm6, ymmword ptr [r8]
 
G_M000_IG07:                ;; offset=0x05E8
       vandnps  ymm9, ymm6, ymmword ptr [rbx]
       vorps    ymm7, ymm9, ymm7
       vmovups  ymmword ptr [r14], ymm7
       vandps   ymm7, ymm6, ymmword ptr [r8+0x20]
       vandnps  ymm9, ymm6, ymmword ptr [rbx+0x20]
       vorps    ymm7, ymm9, ymm7
       vmovups  ymmword ptr [r14+0x20], ymm7
       vandps   ymm7, ymm6, ymmword ptr [r8+0x40]
       vandnps  ymm6, ymm6, ymmword ptr [rbx+0x40]
       vorps    ymm6, ymm6, ymm7
       vmovups  ymmword ptr [rbx+0x40], ymm6
       vpand    ymm6, ymm3, ymm1
       mov      rbx, rsi
       vpor     ymm1, ymm6, ymmword ptr [rbx+0xE0]
       vmovups  ymmword ptr [rbx+0xE0], ymm1
       vandps   ymm1, ymm6, ymmword ptr [rdx]
       vandnps  ymm7, ymm6, ymmword ptr [rbx]
       vorps    ymm1, ymm7, ymm1
       vmovups  ymmword ptr [rbx], ymm1
       vandps   ymm1, ymm6, ymmword ptr [rdx+0x20]
       vandnps  ymm7, ymm6, ymmword ptr [rbx+0x20]
       vorps    ymm1, ymm7, ymm1
       vmovups  ymmword ptr [rbx+0x20], ymm1
       vandps   ymm1, ymm6, ymmword ptr [rdx+0x40]
       vandnps  ymm7, ymm6, ymmword ptr [rbx+0x40]
       vorps    ymm1, ymm7, ymm1
       vmovups  ymmword ptr [rbx+0x40], ymm1
       mov      rbx, rbp
       vandps   ymm1, ymm6, ymmword ptr [r8]
       vandnps  ymm7, ymm6, ymmword ptr [rbp]
       vorps    ymm1, ymm7, ymm1
       vmovups  ymmword ptr [rbx], ymm1
       vandps   ymm1, ymm6, ymmword ptr [r8+0x20]
       vandnps  ymm7, ymm6, ymmword ptr [rbp+0x20]
       vorps    ymm1, ymm7, ymm1
       vmovups  ymmword ptr [rbx+0x20], ymm1
       vandps   ymm1, ymm6, ymmword ptr [r8+0x40]
       vandnps  ymm6, ymm6, ymmword ptr [rbp+0x40]
       vorps    ymm1, ymm6, ymm1
       vmovups  ymmword ptr [rbp+0x40], ymm1
       vpand    ymm6, ymm3, ymm8
       mov      rbx, rdi
       vpor     ymm1, ymm6, ymmword ptr [rbx+0xE0]
       vmovups  ymmword ptr [rbx+0xE0], ymm1
       vandps   ymm1, ymm6, ymmword ptr [rdx]
       vandnps  ymm3, ymm6, ymmword ptr [rbx]
       vorps    ymm1, ymm3, ymm1
       vmovups  ymmword ptr [rbx], ymm1
       vandps   ymm1, ymm6, ymmword ptr [rdx+0x20]
       vandnps  ymm3, ymm6, ymmword ptr [rbx+0x20]
       vorps    ymm1, ymm3, ymm1
       vmovups  ymmword ptr [rbx+0x20], ymm1
       vandps   ymm1, ymm6, ymmword ptr [rdx+0x40]
       vandnps  ymm3, ymm6, ymmword ptr [rbx+0x40]
       vorps    ymm1, ymm3, ymm1
       vmovups  ymmword ptr [rbx+0x40], ymm1
       mov      rdx, r15
       vandps   ymm1, ymm6, ymmword ptr [r8]
       vandnps  ymm3, ymm6, ymmword ptr [r15]
       vorps    ymm1, ymm3, ymm1
 
G_M000_IG08:                ;; offset=0x0707
       vmovups  ymmword ptr [rdx], ymm1
       vandps   ymm1, ymm6, ymmword ptr [r8+0x20]
       vandnps  ymm3, ymm6, ymmword ptr [r15+0x20]
       vorps    ymm1, ymm3, ymm1
       vmovups  ymmword ptr [rdx+0x20], ymm1
       vandps   ymm1, ymm6, ymmword ptr [r8+0x40]
       vandnps  ymm3, ymm6, ymmword ptr [r15+0x40]
       vorps    ymm1, ymm3, ymm1
       vmovups  ymmword ptr [r15+0x40], ymm1
 
G_M000_IG09:                ;; offset=0x0736
       mov      rdx, rsi
       vmovups  ymm1, ymmword ptr [rdx]
       vmovups  ymm3, ymmword ptr [rcx]
       vmovups  ymmword ptr [rsp+0x40], ymm3
       vsubps   ymm7, ymm1, ymm3
       vmovups  ymm6, ymmword ptr [rdx+0x20]
       vmovups  ymmword ptr [rsp+0xA0], ymm6
       vmovups  ymm8, ymmword ptr [rcx+0x20]
       vmovups  ymmword ptr [rsp+0x120], ymm8
       vsubps   ymm9, ymm6, ymm8
       vmovups  ymm10, ymmword ptr [rdx+0x40]
       vmovups  ymmword ptr [rsp+0x80], ymm10
       vmovups  ymm11, ymmword ptr [rcx+0x40]
       vmovups  ymmword ptr [rsp+0x100], ymm11
       vsubps   ymm12, ymm10, ymm11
       vmovups  ymm13, ymmword ptr [rdi]
       vmovups  ymmword ptr [rsp+0x20], ymm13
       vsubps   ymm14, ymm3, ymm13
       vmovups  ymm15, ymmword ptr [rdi+0x20]
       vmovups  ymmword ptr [rsp+0xE0], ymm15
       vsubps   ymm0, ymm8, ymm15
       vmovups  ymm10, ymmword ptr [rdi+0x40]
       vmovups  ymmword ptr [rsp+0xC0], ymm10
       vsubps   ymm6, ymm11, ymm10
       vsubps   ymm1, ymm13, ymm1
       vmovups  ymmword ptr [rsp+0x800], ymm1
       vsubps   ymm15, ymm15, ymmword ptr [rsp+0xA0]
       vmovups  ymmword ptr [rsp+0x7E0], ymm15
       vsubps   ymm10, ymm10, ymmword ptr [rsp+0x80]
       vmovups  ymmword ptr [rsp+0x7C0], ymm10
       vmulps   ymm13, ymm9, ymm6
       vmulps   ymm11, ymm12, ymm0
       vsubps   ymm11, ymm13, ymm11
       vmulps   ymm13, ymm12, ymm14
       vmulps   ymm8, ymm7, ymm6
       vsubps   ymm8, ymm13, ymm8
       vmulps   ymm13, ymm7, ymm0
       vmulps   ymm3, ymm9, ymm14
       vsubps   ymm3, ymm13, ymm3
       vmulps   ymm13, ymm11, ymm11
       vmovups  ymmword ptr [rsp+0xC90], ymm13
       vmulps   ymm13, ymm8, ymm8
       vaddps   ymm13, ymm13, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm13
       vmulps   ymm13, ymm3, ymm3
       vaddps   ymm13, ymm13, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC20], ymm13
       vmovups  ymm13, ymmword ptr [rsp+0x40]
       vsubps   ymm13, ymm13, ymm2
       vmovups  ymmword ptr [rsp+0x7A0], ymm13
       vmovups  ymm10, ymmword ptr [rsp+0x120]
       vsubps   ymm10, ymm10, ymm4
       vmovups  ymmword ptr [rsp+0x780], ymm10
       vmovups  ymm15, ymmword ptr [rsp+0x100]
       vsubps   ymm15, ymm15, ymm5
       vmovups  ymmword ptr [rsp+0x760], ymm15
       vmovups  ymm1, ymmword ptr [rsp+0x20]
       vsubps   ymm1, ymm1, ymm2
       vmovups  ymmword ptr [rsp+0x740], ymm1
 
G_M000_IG10:                ;; offset=0x08A8
       vmovups  ymm2, ymmword ptr [rsp+0xE0]
       vsubps   ymm2, ymm2, ymm4
       vmovups  ymmword ptr [rsp+0x720], ymm2
       vmovups  ymm4, ymmword ptr [rsp+0xC0]
       vsubps   ymm4, ymm4, ymm5
       vmovups  ymmword ptr [rsp+0x700], ymm4
       vmulps   ymm5, ymm9, ymm15
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmulps   ymm5, ymm12, ymm10
       vmovups  ymmword ptr [rsp+0xC70], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0xC90]
       vsubps   ymm5, ymm5, ymmword ptr [rsp+0xC70]
       vmovups  ymmword ptr [rsp+0x6E0], ymm5
       vmulps   ymm5, ymm12, ymm13
       vmovups  ymmword ptr [rsp+0xC70], ymm5
       vmulps   ymm5, ymm7, ymm15
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0xC70]
       vsubps   ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0x6C0], ymm5
       vmulps   ymm5, ymm7, ymm10
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmulps   ymm5, ymm9, ymm13
       vmovups  ymmword ptr [rsp+0xC70], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0xC90]
       vsubps   ymm5, ymm5, ymmword ptr [rsp+0xC70]
       vmovups  ymmword ptr [rsp+0x6A0], ymm5
       vmulps   ymm5, ymm0, ymm4
       vmovups  ymmword ptr [rsp+0xC70], ymm5
       vmulps   ymm5, ymm6, ymm2
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0xC70]
       vsubps   ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0x680], ymm5
       vmulps   ymm5, ymm6, ymm1
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmulps   ymm5, ymm14, ymm4
       vmovups  ymmword ptr [rsp+0xC70], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0xC90]
       vsubps   ymm5, ymm5, ymmword ptr [rsp+0xC70]
       vmovups  ymmword ptr [rsp+0x660], ymm5
       vmulps   ymm5, ymm14, ymm2
       vmovups  ymmword ptr [rsp+0xC70], ymm5
       vmulps   ymm5, ymm0, ymm1
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0xC70]
       vsubps   ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0x640], ymm5
       vmulps   ymm5, ymm11, ymmword ptr [rsp+0x6E0]
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmulps   ymm5, ymm8, ymmword ptr [rsp+0x6C0]
       vaddps   ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmulps   ymm5, ymm3, ymmword ptr [rsp+0x6A0]
       vaddps   ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC00], ymm5
       vmulps   ymm5, ymm11, ymmword ptr [rsp+0x680]
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmulps   ymm5, ymm8, ymmword ptr [rsp+0x660]
 
G_M000_IG11:                ;; offset=0x0A7B
       vaddps   ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmulps   ymm5, ymm3, ymmword ptr [rsp+0x640]
       vaddps   ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xBE0], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0xC20]
       vsubps   ymm5, ymm5, ymmword ptr [rsp+0xBE0]
       vsubps   ymm5, ymm5, ymmword ptr [rsp+0xC00]
       vmovups  ymmword ptr [rsp+0xBC0], ymm5
       vxorps   ymm5, ymm5, ymm5
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0xC00]
       vcmpltps ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xBA0], ymm5
       vxorps   ymm5, ymm5, ymm5
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0xBC0]
       vcmpltps ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xB80], ymm5
       vxorps   ymm5, ymm5, ymm5
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0xBE0]
       vcmpltps ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xB60], ymm5
       vmulps   ymm5, ymm7, ymm7
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmulps   ymm5, ymm9, ymm9
       vaddps   ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm5
       vmulps   ymm5, ymm12, ymm12
       vaddps   ymm5, ymm5, ymmword ptr [rsp+0xC90]
       vmovups  ymm4, ymmword ptr [rsp+0x800]
       vmulps   ymm4, ymm4, ymm4
       vmovups  ymmword ptr [rsp+0xC90], ymm4
       vmovups  ymm4, ymmword ptr [rsp+0x7E0]
       vmulps   ymm4, ymm4, ymm4
       vaddps   ymm4, ymm4, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm4
       vmovups  ymm4, ymmword ptr [rsp+0x7C0]
       vmulps   ymm4, ymm4, ymm4
       vaddps   ymm4, ymm4, ymmword ptr [rsp+0xC90]
       vmulps   ymm2, ymm14, ymm14
       vmovups  ymmword ptr [rsp+0xC90], ymm2
       vmulps   ymm2, ymm0, ymm0
       vaddps   ymm2, ymm2, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm2
       vmulps   ymm2, ymm6, ymm6
       vaddps   ymm2, ymm2, ymmword ptr [rsp+0xC90]
       vcmpeqps ymm1, ymm5, ymm4
       vmovups  ymmword ptr [rsp+0xC90], ymm1
       vxorps   ymm1, ymm1, ymm1
       vpcmpgtd ymm1, ymm1, ymm4
       vandps   ymm1, ymm1, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm1
       vcmpneqps ymm1, ymm5, ymm5
       vorps    ymm1, ymm1, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm1
       vcmpltps ymm1, ymm4, ymm5
       vorps    ymm1, ymm1, ymmword ptr [rsp+0xC90]
 
G_M000_IG12:                ;; offset=0x0C42
       vblendvps ymm1, ymm4, ymm5, ymm1
       vcmpeqps ymm15, ymm1, ymm2
       vmovups  ymmword ptr [rsp+0xC90], ymm15
       vxorps   ymm15, ymm15, ymm15
       vpcmpgtd ymm15, ymm15, ymm2
       vandps   ymm15, ymm15, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm15
       vcmpneqps ymm15, ymm1, ymm1
       vorps    ymm15, ymm15, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm15
       vcmpltps ymm15, ymm2, ymm1
       vorps    ymm15, ymm15, ymmword ptr [rsp+0xC90]
       vblendvps ymm1, ymm2, ymm1, ymm15
       vmovups  ymmword ptr [rsp+0xB40], ymm1
       vmulps   ymm15, ymm1, ymmword ptr [reloc @RWD00]
       vmovups  ymm1, ymmword ptr [rsp+0xC20]
       vcmpleps ymm15, ymm1, ymm15
       vmovups  ymmword ptr [rsp+0xB20], ymm15
       vmovups  ymm1, ymmword ptr [rsp+0xB40]
       vcmpltps ymm1, ymm1, ymmword ptr [reloc @RWD32]
       vmovups  ymmword ptr [rsp+0xB00], ymm1
       vpandn   ymm15, ymm1, ymm15
       vmovups  ymmword ptr [rsp+0xAE0], ymm15
       vmulps   ymm15, ymm11, ymmword ptr [rax]
       vmulps   ymm1, ymm8, ymmword ptr [rax+0x20]
       vaddps   ymm1, ymm1, ymm15
       vmulps   ymm15, ymm3, ymmword ptr [rax+0x40]
       vaddps   ymm1, ymm15, ymm1
       vxorps   ymm15, ymm15, ymm15
       vcmpltps ymm1, ymm1, ymm15
       vbroadcastss ymm15, dword ptr [reloc @RWD64]
       vxorps   ymm10, ymm15, ymm11
       vandps   ymm10, ymm10, ymm1
       vandnps  ymm11, ymm1, ymm11
       vorps    ymm11, ymm11, ymm10
       vxorps   ymm10, ymm15, ymm8
       vandps   ymm10, ymm10, ymm1
       vandnps  ymm8, ymm1, ymm8
       vorps    ymm8, ymm8, ymm10
       vxorps   ymm10, ymm15, ymm3
       vandps   ymm10, ymm10, ymm1
       vandnps  ymm3, ymm1, ymm3
       vorps    ymm3, ymm3, ymm10
       vmovups  ymm1, ymmword ptr [rsp+0xB80]
       vpor     ymm1, ymm1, ymmword ptr [rsp+0xB60]
       vpor     ymm1, ymm1, ymmword ptr [rsp+0xBA0]
       vmovups  ymmword ptr [rsp+0xAC0], ymm1
       vxorps   ymm10, ymm15, ymm13
       vmovups  ymmword ptr [rsp+0x620], ymm10
       vxorps   ymm1, ymm15, ymmword ptr [rsp+0x780]
       vmovups  ymmword ptr [rsp+0x600], ymm1
       vxorps   ymm1, ymm15, ymmword ptr [rsp+0x760]
       vmovups  ymmword ptr [rsp+0x5E0], ymm1
       vbroadcastss ymm1, dword ptr [reloc @RWD68]
       vmovups  ymmword ptr [rsp], ymm1
       vmovups  ymmword ptr [rsp+0xAA0], ymm1
       vmovups  ymm10, ymmword ptr [r9]
       vandps   ymm1, ymm10, ymmword ptr [rcx+0xC0]
       vmovups  ymmword ptr [rsp+0xC90], ymm1
 
G_M000_IG13:                ;; offset=0x0DD3
       vbroadcastss ymm1, dword ptr [reloc @RWD72]
       vmovups  ymmword ptr [rsp+0x60], ymm1
       vandnps  ymm10, ymm10, ymm1
       vorps    ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rcx+0xC0], ymm10
       vmovups  ymm10, ymmword ptr [r9]
       vandps   ymm1, ymm10, ymmword ptr [rcx+0x1C0]
       vmovups  ymmword ptr [rsp+0xC90], ymm1
       vxorps   ymm1, ymm1, ymm1
       vandnps  ymm1, ymm10, ymm1
       vorps    ymm1, ymm1, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rcx+0x1C0], ymm1
       vmovups  ymm1, ymmword ptr [r9]
       vandps   ymm10, ymm1, ymmword ptr [rcx+0x2C0]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vxorps   ymm10, ymm10, ymm10
       vandnps  ymm1, ymm1, ymm10
       vorps    ymm1, ymm1, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rcx+0x2C0], ymm1
       vmovups  ymm1, ymmword ptr [r9]
       vandps   ymm10, ymm1, ymmword ptr [rcx+0x300]
       vandnps  ymm1, ymm1, ymmword ptr [rsp+0x60]
       vorps    ymm1, ymm1, ymm10
       vmovups  ymmword ptr [rcx+0x300], ymm1
       vmulps   ymm1, ymm13, ymm13
       vmovups  ymm10, ymmword ptr [rsp+0x780]
       vmulps   ymm10, ymm10, ymm10
       vaddps   ymm1, ymm10, ymm1
       vmovups  ymm10, ymmword ptr [rsp+0x760]
       vmulps   ymm10, ymm10, ymm10
       vaddps   ymm1, ymm10, ymm1
       vcmpltps ymm1, ymm1, ymmword ptr [rsp+0xC40]
       vpand    ymm1, ymm1, ymmword ptr [rsp+0xB00]
       vpor     ymm1, ymm1, ymmword ptr [r9]
       vmovups  ymmword ptr [r9], ymm1
       vmovups  ymm1, ymmword ptr [rsp+0xAC0]
       vpor     ymm1, ymm1, ymmword ptr [rsp+0xAE0]
       vmovups  ymm10, ymmword ptr [r9]
       vpandn   ymm1, ymm10, ymm1
       vxorps   ymm10, ymm10, ymm10
       vpcmpgtd ymm10, ymm10, ymm1
       vptest   ymm10, ymm10
       je       G_M000_IG21
 
G_M000_IG14:                ;; offset=0x0EEC
       vmovups  ymm10, ymmword ptr [rsp+0x60]
       vdivps   ymm10, ymm10, ymm5
       vmovups  ymmword ptr [rsp+0xA80], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x60]
       vdivps   ymm10, ymm10, ymm4
       vmovups  ymmword ptr [rsp+0xA60], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x60]
       vdivps   ymm10, ymm10, ymm2
       vmovups  ymmword ptr [rsp+0xA40], ymm10
       vmovups  ymm10, ymmword ptr [rsi]
       vsubps   ymm10, ymm10, ymmword ptr [rsp+0x860]
       vmovups  ymmword ptr [rsp+0x480], ymm10
       vmovups  ymm10, ymmword ptr [rsi+0x20]
       vsubps   ymm10, ymm10, ymmword ptr [rsp+0x840]
       vmovups  ymmword ptr [rsp+0x460], ymm10
       vmovups  ymm10, ymmword ptr [rsi+0x40]
       vsubps   ymm10, ymm10, ymmword ptr [rsp+0x820]
       vmovups  ymmword ptr [rsp+0x440], ymm10
       vmulps   ymm10, ymm7, ymm13
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmulps   ymm10, ymm9, ymmword ptr [rsp+0x780]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmulps   ymm10, ymm12, ymmword ptr [rsp+0x760]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xA20], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x800]
       vmulps   ymm10, ymm10, ymmword ptr [rsp+0x480]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x7E0]
       vmulps   ymm10, ymm10, ymmword ptr [rsp+0x460]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x7C0]
       vmulps   ymm10, ymm10, ymmword ptr [rsp+0x440]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xA00], ymm10
       vmulps   ymm10, ymm14, ymmword ptr [rsp+0x740]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmulps   ymm10, ymm0, ymmword ptr [rsp+0x720]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmulps   ymm10, ymm6, ymmword ptr [rsp+0x700]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0x9E0], ymm10
       vxorps   ymm10, ymm15, ymmword ptr [rsp+0xA20]
       vmovups  ymmword ptr [rsp+0x880], ymm10
       vcmpeqps ymm10, ymm5, ymm10
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vxorps   ymm10, ymm10, ymm10
       vpcmpgtd ymm10, ymm10, ymm5
       vandps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vcmpneqps ymm10, ymm5, ymm5
       vorps    ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vcmpltps ymm10, ymm5, ymmword ptr [rsp+0x880]
       vorps    ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymm13, ymmword ptr [rsp+0x880]
 
G_M000_IG15:                ;; offset=0x10C7
       vblendvps ymm5, ymm13, ymm5, ymm10
       vxorps   ymm10, ymm10, ymm10
       vcmpeqps ymm10, ymm10, ymm5
       vxorps   ymm13, ymm13, ymm13
       vpcmpgtd ymm13, ymm13, ymm5
       vandps   ymm10, ymm13, ymm10
       vxorps   ymm13, ymm13, ymm13
       vcmpltps ymm13, ymm5, ymm13
       vorps    ymm10, ymm13, ymm10
       vandnps  ymm5, ymm10, ymm5
       vmovups  ymmword ptr [rsp+0x9C0], ymm5
       vxorps   ymm10, ymm15, ymmword ptr [rsp+0xA00]
       vcmpeqps ymm13, ymm4, ymm10
       vxorps   ymm5, ymm5, ymm5
       vpcmpgtd ymm5, ymm5, ymm4
       vandps   ymm5, ymm5, ymm13
       vcmpneqps ymm13, ymm4, ymm4
       vorps    ymm5, ymm13, ymm5
       vcmpltps ymm13, ymm4, ymm10
       vorps    ymm5, ymm13, ymm5
       vblendvps ymm5, ymm10, ymm4, ymm5
       vxorps   ymm10, ymm10, ymm10
       vcmpeqps ymm10, ymm10, ymm5
       vxorps   ymm13, ymm13, ymm13
       vpcmpgtd ymm13, ymm13, ymm5
       vandps   ymm10, ymm13, ymm10
       vxorps   ymm13, ymm13, ymm13
       vcmpltps ymm13, ymm5, ymm13
       vorps    ymm10, ymm13, ymm10
       vandnps  ymm5, ymm10, ymm5
       vmovups  ymmword ptr [rsp+0x9A0], ymm5
       vxorps   ymm10, ymm15, ymmword ptr [rsp+0x9E0]
       vcmpeqps ymm13, ymm2, ymm10
       vxorps   ymm5, ymm5, ymm5
       vpcmpgtd ymm5, ymm5, ymm2
       vandps   ymm5, ymm5, ymm13
       vcmpneqps ymm13, ymm2, ymm2
       vorps    ymm5, ymm13, ymm5
       vcmpltps ymm13, ymm2, ymm10
       vorps    ymm5, ymm13, ymm5
       vblendvps ymm5, ymm10, ymm2, ymm5
       vxorps   ymm10, ymm10, ymm10
       vcmpeqps ymm10, ymm10, ymm5
       vxorps   ymm13, ymm13, ymm13
       vpcmpgtd ymm13, ymm13, ymm5
       vandps   ymm10, ymm13, ymm10
       vxorps   ymm13, ymm13, ymm13
       vcmpltps ymm13, ymm5, ymm13
       vorps    ymm10, ymm13, ymm10
       vandnps  ymm5, ymm10, ymm5
       vmovups  ymm10, ymmword ptr [rsp+0x9C0]
       vmulps   ymm10, ymm10, ymmword ptr [rsp+0xA80]
       vmovups  ymmword ptr [rsp+0x980], ymm10
       vmovups  ymm13, ymmword ptr [rsp+0x9A0]
       vmulps   ymm13, ymm13, ymmword ptr [rsp+0xA60]
       vmovups  ymmword ptr [rsp+0x960], ymm13
       vmulps   ymm5, ymm5, ymmword ptr [rsp+0xA40]
       vmovups  ymmword ptr [rsp+0x940], ymm5
       vmulps   ymm5, ymm7, ymm10
 
G_M000_IG16:                ;; offset=0x121A
       vmovups  ymmword ptr [rsp+0x420], ymm5
       vmulps   ymm5, ymm9, ymm10
       vmovups  ymmword ptr [rsp+0x400], ymm5
       vmulps   ymm5, ymm12, ymm10
       vmovups  ymmword ptr [rsp+0x3E0], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0x800]
       vmulps   ymm5, ymm5, ymm13
       vmovups  ymmword ptr [rsp+0x3C0], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0x7E0]
       vmulps   ymm5, ymm5, ymm13
       vmovups  ymmword ptr [rsp+0x3A0], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0x7C0]
       vmulps   ymm5, ymm5, ymm13
       vmovups  ymmword ptr [rsp+0x380], ymm5
       vmulps   ymm5, ymm14, ymmword ptr [rsp+0x940]
       vmovups  ymmword ptr [rsp+0x360], ymm5
       vmulps   ymm5, ymm0, ymmword ptr [rsp+0x940]
       vmovups  ymmword ptr [rsp+0x340], ymm5
       vmulps   ymm5, ymm6, ymmword ptr [rsp+0x940]
       vmovups  ymmword ptr [rsp+0x320], ymm5
       vmovups  ymm5, ymmword ptr [rsp+0x7A0]
       vaddps   ymm13, ymm5, ymmword ptr [rsp+0x420]
       vmovups  ymmword ptr [rsp+0x300], ymm13
       vmovups  ymm13, ymmword ptr [rsp+0x780]
       vaddps   ymm10, ymm13, ymmword ptr [rsp+0x400]
       vmovups  ymmword ptr [rsp+0x2E0], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x760]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0x3E0]
       vmovups  ymmword ptr [rsp+0x2C0], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x480]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0x3C0]
       vmovups  ymmword ptr [rsp+0x2A0], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x460]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0x3A0]
       vmovups  ymmword ptr [rsp+0x280], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x440]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0x380]
       vmovups  ymmword ptr [rsp+0x260], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x740]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0x360]
       vmovups  ymmword ptr [rsp+0x240], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x720]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0x340]
       vmovups  ymmword ptr [rsp+0x220], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x700]
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0x320]
       vmovups  ymmword ptr [rsp+0x200], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x300]
       vmulps   ymm10, ymm10, ymm10
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x2E0]
       vmulps   ymm10, ymm10, ymm10
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x2C0]
       vmulps   ymm10, ymm10, ymm10
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0x920], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x2A0]
 
G_M000_IG17:                ;; offset=0x140D
       vmulps   ymm10, ymm10, ymm10
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x280]
       vmulps   ymm10, ymm10, ymm10
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x260]
       vmulps   ymm10, ymm10, ymm10
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0x900], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x240]
       vmulps   ymm10, ymm10, ymm10
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x220]
       vmulps   ymm10, ymm10, ymm10
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vmovups  ymm10, ymmword ptr [rsp+0x200]
       vmulps   ymm10, ymm10, ymm10
       vaddps   ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rsp+0x8E0], ymm10
       vxorps   ymm10, ymm10, ymm10
       vcmpeqps ymm4, ymm10, ymm4
       vxorps   ymm10, ymm10, ymm10
       vcmpeqps ymm2, ymm10, ymm2
       vmovups  ymm10, ymmword ptr [rsp+0x920]
       vcmpltps ymm10, ymm10, ymmword ptr [rsp+0x900]
       vpor     ymm4, ymm10, ymm4
       vmovups  ymmword ptr [rsp+0x8C0], ymm4
       vmovups  ymm10, ymmword ptr [rsp+0x920]
       vcmpltps ymm10, ymm10, ymmword ptr [rsp+0x8E0]
       vpor     ymm10, ymm10, ymm2
       vmovups  ymm4, ymmword ptr [rsp+0x900]
       vcmpltps ymm4, ymm4, ymmword ptr [rsp+0x8E0]
       vpor     ymm2, ymm4, ymm2
       vmovups  ymm4, ymmword ptr [rsp+0x8C0]
       vpand    ymm4, ymm4, ymm10
       vpandn   ymm2, ymm4, ymm2
       vandps   ymm10, ymm4, ymmword ptr [rsp+0x920]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vandps   ymm10, ymm2, ymmword ptr [rsp+0x900]
       vmovups  ymmword ptr [rsp+0xC70], ymm10
       vandnps  ymm10, ymm2, ymmword ptr [rsp+0x8E0]
       vorps    ymm10, ymm10, ymmword ptr [rsp+0xC70]
       vandnps  ymm10, ymm4, ymm10
       vorps    ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vcmpleps ymm10, ymm10, ymmword ptr [rsp+0xC40]
       vpand    ymm10, ymm10, ymm1
       vpor     ymm10, ymm10, ymmword ptr [r9]
       vmovups  ymmword ptr [r9], ymm10
       vandps   ymm10, ymm4, ymmword ptr [rsp+0x980]
       vmovups  ymmword ptr [rsp+0xC90], ymm10
       vandps   ymm10, ymm2, ymmword ptr [rsp+0x960]
       vmovups  ymmword ptr [rsp+0xC70], ymm10
       vandnps  ymm10, ymm2, ymmword ptr [rsp+0x940]
       vorps    ymm10, ymm10, ymmword ptr [rsp+0xC70]
       vandnps  ymm10, ymm4, ymm10
       vorps    ymm10, ymm10, ymmword ptr [rsp+0xC90]
       vandps   ymm7, ymm7, ymm4
 
G_M000_IG18:                ;; offset=0x15CA
       vandnps  ymm14, ymm4, ymm14
       vorps    ymm7, ymm14, ymm7
       vandps   ymm9, ymm9, ymm4
       vandnps  ymm0, ymm4, ymm0
       vorps    ymm0, ymm0, ymm9
       vandps   ymm9, ymm12, ymm4
       vandnps  ymm6, ymm4, ymm6
       vorps    ymm6, ymm6, ymm9
       vandps   ymm9, ymm5, ymm4
       vandnps  ymm12, ymm4, ymmword ptr [rsp+0x740]
       vorps    ymm9, ymm12, ymm9
       vmovups  ymmword ptr [rsp+0x1C0], ymm9
       vandps   ymm12, ymm13, ymm4
       vandnps  ymm14, ymm4, ymmword ptr [rsp+0x720]
       vorps    ymm12, ymm14, ymm12
       vmovups  ymmword ptr [rsp+0x1A0], ymm12
       vmovups  ymm14, ymmword ptr [rsp+0x760]
       vandps   ymm12, ymm14, ymm4
       vandnps  ymm9, ymm4, ymmword ptr [rsp+0x700]
       vorps    ymm9, ymm9, ymm12
       vandps   ymm12, ymm2, ymmword ptr [rsp+0x800]
       vandnps  ymm7, ymm2, ymm7
       vorps    ymm7, ymm7, ymm12
       vandps   ymm12, ymm2, ymmword ptr [rsp+0x7E0]
       vandnps  ymm0, ymm2, ymm0
       vorps    ymm0, ymm0, ymm12
       vandps   ymm12, ymm2, ymmword ptr [rsp+0x7C0]
       vandnps  ymm6, ymm2, ymm6
       vorps    ymm6, ymm6, ymm12
       vmovups  ymmword ptr [rsp+0x1E0], ymm6
       vandps   ymm12, ymm2, ymmword ptr [rsp+0x480]
       vandnps  ymm6, ymm2, ymmword ptr [rsp+0x1C0]
       vorps    ymm6, ymm6, ymm12
       vmovups  ymmword ptr [rsp+0x1C0], ymm6
       vandps   ymm12, ymm2, ymmword ptr [rsp+0x460]
       vandnps  ymm6, ymm2, ymmword ptr [rsp+0x1A0]
       vorps    ymm12, ymm6, ymm12
       vandps   ymm6, ymm2, ymmword ptr [rsp+0x440]
       vandnps  ymm9, ymm2, ymm9
       vorps    ymm9, ymm9, ymm6
       vxorps   ymm6, ymm15, ymm10
       vmulps   ymm7, ymm6, ymm7
       vmulps   ymm0, ymm6, ymm0
       vmulps   ymm6, ymm6, ymmword ptr [rsp+0x1E0]
       vsubps   ymm7, ymm7, ymmword ptr [rsp+0x1C0]
       vmovups  ymmword ptr [rsp+0x180], ymm7
       vsubps   ymm0, ymm0, ymm12
       vmovups  ymmword ptr [rsp+0x160], ymm0
       vsubps   ymm6, ymm6, ymm9
       vmovups  ymmword ptr [rsp+0x140], ymm6
       vxorps   ymm9, ymm9, ymm9
       vcmpeqps ymm9, ymm9, ymm10
       vmovups  ymm12, ymmword ptr [rsp+0x60]
       vcmpeqps ymm15, ymm12, ymm10
       vmovups  ymm6, ymmword ptr [rsp]
       vpand    ymm0, ymm6, ymm9
       vpand    ymm7, ymm15, ymmword ptr [reloc @RWD96]
       vpandn   ymm6, ymm15, ymmword ptr [reloc @RWD128]
       vpor     ymm6, ymm6, ymm7
 
G_M000_IG19:                ;; offset=0x173F
       vpandn   ymm6, ymm9, ymm6
       vpor     ymm0, ymm6, ymm0
       vpand    ymm0, ymm0, ymm4
       vmovups  ymmword ptr [rsp+0xC90], ymm0
       vpand    ymm6, ymm9, ymmword ptr [reloc @RWD96]
       vpand    ymm7, ymm15, ymmword ptr [reloc @RWD160]
       vpandn   ymm0, ymm15, ymmword ptr [reloc @RWD192]
       vpor     ymm0, ymm0, ymm7
       vpandn   ymm0, ymm9, ymm0
       vpor     ymm0, ymm0, ymm6
       vpand    ymm0, ymm0, ymm2
       vpand    ymm6, ymm9, ymmword ptr [reloc @RWD160]
       vmovups  ymm7, ymmword ptr [rsp]
       vpand    ymm7, ymm7, ymm15
       vpandn   ymm15, ymm15, ymmword ptr [reloc @RWD224]
       vpor     ymm7, ymm15, ymm7
       vpandn   ymm7, ymm9, ymm7
       vpor     ymm6, ymm7, ymm6
       vpandn   ymm6, ymm2, ymm6
       vpor     ymm0, ymm6, ymm0
       vpandn   ymm0, ymm4, ymm0
       vpor     ymm0, ymm0, ymmword ptr [rsp+0xC90]
       vpand    ymm0, ymm0, ymm1
       vpandn   ymm6, ymm1, ymmword ptr [rsp+0xAA0]
       vpor     ymm0, ymm6, ymm0
       vandps   ymm6, ymm1, ymmword ptr [rsp+0x180]
       vandnps  ymm7, ymm1, ymmword ptr [rsp+0x620]
       vorps    ymm6, ymm7, ymm6
       vandps   ymm7, ymm1, ymmword ptr [rsp+0x160]
       vandnps  ymm9, ymm1, ymmword ptr [rsp+0x600]
       vorps    ymm7, ymm9, ymm7
       vandps   ymm9, ymm1, ymmword ptr [rsp+0x140]
       vandnps  ymm15, ymm1, ymmword ptr [rsp+0x5E0]
       vorps    ymm9, ymm15, ymm9
       vsubps   ymm15, ymm12, ymm10
       vmovups  ymmword ptr [rsp+0x8A0], ymm15
       vandps   ymm15, ymm4, ymm15
       vmovups  ymmword ptr [rsp+0xC90], ymm15
       vxorps   ymm15, ymm15, ymm15
       vandps   ymm15, ymm15, ymm2
       vmovups  ymmword ptr [rsp+0xC70], ymm15
       vandnps  ymm15, ymm2, ymm10
       vorps    ymm15, ymm15, ymmword ptr [rsp+0xC70]
       vandnps  ymm15, ymm4, ymm15
       vorps    ymm15, ymm15, ymmword ptr [rsp+0xC90]
       vandps   ymm15, ymm15, ymm1
       vmovups  ymmword ptr [rsp+0xC90], ymm15
       vandnps  ymm15, ymm1, ymmword ptr [rcx+0xC0]
       vorps    ymm15, ymm15, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rcx+0xC0], ymm15
       vandps   ymm15, ymm4, ymm10
       vmovups  ymmword ptr [rsp+0xC90], ymm15
       vandps   ymm15, ymm2, ymmword ptr [rsp+0x8A0]
       vmovups  ymmword ptr [rsp+0xC70], ymm15
       vxorps   ymm15, ymm15, ymm15
       vandnps  ymm15, ymm2, ymm15
       vorps    ymm15, ymm15, ymmword ptr [rsp+0xC70]
       vandnps  ymm15, ymm4, ymm15
       vorps    ymm15, ymm15, ymmword ptr [rsp+0xC90]
 
G_M000_IG20:                ;; offset=0x18BC
       vandps   ymm15, ymm15, ymm1
       vmovups  ymmword ptr [rsp+0xC90], ymm15
       vandnps  ymm15, ymm1, ymmword ptr [rcx+0x1C0]
       vorps    ymm15, ymm15, ymmword ptr [rsp+0xC90]
       vmovups  ymmword ptr [rcx+0x1C0], ymm15
       vxorps   ymm15, ymm15, ymm15
       vandps   ymm15, ymm15, ymm4
       vandps   ymm10, ymm2, ymm10
       vandnps  ymm2, ymm2, ymmword ptr [rsp+0x8A0]
       vorps    ymm2, ymm2, ymm10
       vandnps  ymm2, ymm4, ymm2
       vorps    ymm2, ymm2, ymm15
       vandps   ymm2, ymm2, ymm1
       vandnps  ymm1, ymm1, ymmword ptr [rcx+0x2C0]
       vorps    ymm1, ymm1, ymm2
       vmovups  ymmword ptr [rcx+0x2C0], ymm1
       vmovups  ymmword ptr [rsp+0x620], ymm6
       vmovups  ymmword ptr [rsp+0x600], ymm7
       vmovups  ymmword ptr [rsp+0x5E0], ymm9
       vmovups  ymmword ptr [rsp+0xAA0], ymm0
       vmovaps  ymm13, ymm5
 
G_M000_IG21:                ;; offset=0x1947
       vpcmpeqd ymm1, ymm1, ymm1
       vpxor    ymm1, ymm1, ymmword ptr [rsp+0xB20]
       vmovups  ymm2, ymmword ptr [rsp+0xAC0]
       vpandn   ymm1, ymm2, ymm1
       vmovups  ymm2, ymmword ptr [r9]
       vpandn   ymm1, ymm2, ymm1
       vxorps   ymm2, ymm2, ymm2
       vpcmpgtd ymm2, ymm2, ymm1
       vptest   ymm2, ymm2
       je       G_M000_IG23
 
G_M000_IG22:                ;; offset=0x197D
       vmulps   ymm2, ymm11, ymm13
       vmulps   ymm4, ymm8, ymmword ptr [rsp+0x780]
       vaddps   ymm2, ymm4, ymm2
       vmulps   ymm4, ymm3, ymmword ptr [rsp+0x760]
       vaddps   ymm2, ymm4, ymm2
       vmulps   ymm2, ymm2, ymm2
       vmovups  ymm5, ymmword ptr [rsp+0xC20]
       vmulps   ymm4, ymm5, ymmword ptr [rsp+0xC40]
       vcmpltps ymm2, ymm2, ymm4
       vpand    ymm2, ymm2, ymm1
       vpor     ymm2, ymm2, ymmword ptr [r9]
       vmovups  ymmword ptr [r9], ymm2
       vandps   ymm2, ymm11, ymm1
       vandnps  ymm4, ymm1, ymmword ptr [rsp+0x620]
       vorps    ymm6, ymm4, ymm2
       vandps   ymm2, ymm8, ymm1
       vandnps  ymm4, ymm1, ymmword ptr [rsp+0x600]
       vorps    ymm7, ymm4, ymm2
       vandps   ymm2, ymm3, ymm1
       vandnps  ymm3, ymm1, ymmword ptr [rsp+0x5E0]
       vorps    ymm9, ymm3, ymm2
       vpand    ymm2, ymm1, ymmword ptr [reloc @RWD256]
       vpandn   ymm0, ymm1, ymmword ptr [rsp+0xAA0]
       vpor     ymm0, ymm0, ymm2
       vandps   ymm2, ymm1, ymmword ptr [rsp+0xBC0]
       vandnps  ymm3, ymm1, ymmword ptr [rcx+0xC0]
       vorps    ymm2, ymm3, ymm2
       vmovups  ymmword ptr [rcx+0xC0], ymm2
       vandps   ymm2, ymm1, ymmword ptr [rsp+0xBE0]
       vandnps  ymm3, ymm1, ymmword ptr [rcx+0x1C0]
       vorps    ymm2, ymm3, ymm2
       vmovups  ymmword ptr [rcx+0x1C0], ymm2
       vandps   ymm2, ymm1, ymmword ptr [rsp+0xC00]
       vandnps  ymm3, ymm1, ymmword ptr [rcx+0x2C0]
       vorps    ymm2, ymm3, ymm2
       vmovups  ymmword ptr [rcx+0x2C0], ymm2
       vandps   ymm2, ymm1, ymm5
       vandnps  ymm3, ymm1, ymmword ptr [rcx+0x300]
       vorps    ymm2, ymm3, ymm2
       vmovups  ymmword ptr [rcx+0x300], ymm2
       vmovups  ymmword ptr [rsp+0x620], ymm6
       vmovups  ymmword ptr [rsp+0x600], ymm7
       vmovups  ymmword ptr [rsp+0x5E0], ymm9
       vmovups  ymmword ptr [rsp+0xAA0], ymm0
 
G_M000_IG23:                ;; offset=0x1AA0
       vmovups  ymm2, ymmword ptr [rsp]
       vmovups  ymm0, ymmword ptr [rsp+0xAA0]
       vpand    ymm2, ymm2, ymm0
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm2, ymm2, ymm3
       vmovups  ymmword ptr [rcx+0xE0], ymm2
       vpand    ymm2, ymm0, ymmword ptr [reloc @RWD96]
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm2, ymm2, ymm3
       vmovups  ymmword ptr [rcx+0x1E0], ymm2
       vpand    ymm0, ymm0, ymmword ptr [reloc @RWD160]
       vxorps   ymm2, ymm2, ymm2
       vpcmpgtd ymm0, ymm0, ymm2
       vmovups  ymmword ptr [rcx+0x2E0], ymm0
       vxorps   ymm0, ymm0, ymm0
       vpcmpeqd ymm0, ymm0, ymmword ptr [r9]
       vptest   ymm0, ymm0
       je       G_M000_IG25
 
G_M000_IG24:                ;; offset=0x1B06
       vbroadcastss ymm0, dword ptr [reloc @RWD288]
       vmovups  ymm6, ymmword ptr [rsp+0x620]
       vmulps   ymm2, ymm0, ymm6
       vmovups  ymm7, ymmword ptr [rsp+0x600]
       vmulps   ymm3, ymm0, ymm7
       vmovups  ymm9, ymmword ptr [rsp+0x5E0]
       vmulps   ymm0, ymm0, ymm9
       vmovups  ymm4, ymmword ptr [rsp+0x860]
       vaddps   ymm2, ymm4, ymm2
       vmovups  ymm4, ymmword ptr [rsp+0x840]
       vaddps   ymm3, ymm4, ymm3
       vmovups  ymm5, ymmword ptr [rsp+0x820]
       vaddps   ymm0, ymm5, ymm0
       vmovups  ymm4, ymmword ptr [r11]
       vxorps   ymm5, ymm5, ymm5
       vcmpleps ymm4, ymm4, ymm5
       vpor     ymm1, ymm4, ymm1
       vandps   ymm4, ymm1, ymm6
       vandnps  ymm2, ymm1, ymm2
       vorps    ymm6, ymm2, ymm4
       vandps   ymm2, ymm1, ymm7
       vandnps  ymm3, ymm1, ymm3
       vorps    ymm7, ymm3, ymm2
       vandps   ymm2, ymm1, ymm9
       vandnps  ymm0, ymm1, ymm0
       vorps    ymm9, ymm0, ymm2
       vmulps   ymm0, ymm6, ymm6
       vmulps   ymm1, ymm7, ymm7
       vaddps   ymm0, ymm1, ymm0
       vmulps   ymm1, ymm9, ymm9
       vaddps   ymm0, ymm1, ymm0
       vsqrtps  ymm0, ymm0
       vmovups  ymm12, ymmword ptr [rsp+0x60]
       vdivps   ymm0, ymm12, ymm0
       vmulps   ymm1, ymm0, ymm6
       vmovups  ymmword ptr [r10], ymm1
       vmulps   ymm1, ymm0, ymm7
       vmovups  ymmword ptr [r10+0x20], ymm1
       vmulps   ymm0, ymm0, ymm9
       vmovups  ymmword ptr [r10+0x40], ymm0
 
G_M000_IG25:                ;; offset=0x1BD6
       vzeroupper 
       vmovaps  xmm6, xmmword ptr [rsp+0xD40]
       vmovaps  xmm7, xmmword ptr [rsp+0xD30]
       vmovaps  xmm8, xmmword ptr [rsp+0xD20]
       vmovaps  xmm9, xmmword ptr [rsp+0xD10]
       vmovaps  xmm10, xmmword ptr [rsp+0xD00]
       vmovaps  xmm11, xmmword ptr [rsp+0xCF0]
       vmovaps  xmm12, xmmword ptr [rsp+0xCE0]
       vmovaps  xmm13, xmmword ptr [rsp+0xCD0]
       vmovaps  xmm14, xmmword ptr [rsp+0xCC0]
       vmovaps  xmm15, xmmword ptr [rsp+0xCB0]
       add      rsp, 0xD58
       pop      rbx
       pop      rbp
       pop      rsi
       pop      rdi
       pop      r12
       pop      r13
       pop      r14
       pop      r15
       ret      
 
RWD00  	dq	2EDBE6FF2EDBE6FFh, 2EDBE6FF2EDBE6FFh, 2EDBE6FF2EDBE6FFh, 2EDBE6FF2EDBE6FFh
RWD32  	dq	283424DC283424DCh, 283424DC283424DCh, 283424DC283424DCh, 283424DC283424DCh
RWD64  	dd	80000000h		;        -0
RWD68  	dd	00000001h		; 1.4013e-45
RWD72  	dd	3F800000h		;         1
RWD76  	dd	00000000h, 00000000h, 00000000h, 00000000h, 00000000h
RWD96  	dq	0000000200000002h, 0000000200000002h, 0000000200000002h, 0000000200000002h
RWD128 	dq	0000000300000003h, 0000000300000003h, 0000000300000003h, 0000000300000003h
RWD160 	dq	0000000400000004h, 0000000400000004h, 0000000400000004h, 0000000400000004h
RWD192 	dq	0000000600000006h, 0000000600000006h, 0000000600000006h, 0000000600000006h
RWD224 	dq	0000000500000005h, 0000000500000005h, 0000000500000005h, 0000000500000005h
RWD256 	dq	0000000700000007h, 0000000700000007h, 0000000700000007h, 0000000700000007h
RWD288 	dd	40800000h		;         4

; Total bytes of code 7239

; Assembly listing for method AosBaselines.ScalarDepthRefiner`4[BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar,BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar]:GetNextNormal(byref,System.Numerics.Vector3,System.Numerics.Vector3,byref,System.Numerics.Vector3,float,float):System.Numerics.Vector3 (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rsp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 74 single block inlinees; 55 inlinees without PGO data

G_M000_IG01:                ;; offset=0x0000
       push     rsi
       push     rbx
       sub      rsp, 392
       vmovaps  xmmword ptr [rsp+0x170], xmm6
       vmovaps  xmmword ptr [rsp+0x160], xmm7
       vmovaps  xmmword ptr [rsp+0x150], xmm8
       vmovaps  xmmword ptr [rsp+0x140], xmm9
       vmovaps  xmmword ptr [rsp+0x130], xmm10
       vmovaps  xmmword ptr [rsp+0x120], xmm11
       vmovaps  xmmword ptr [rsp+0x110], xmm12
       vmovaps  xmmword ptr [rsp+0x100], xmm13
       vmovaps  xmmword ptr [rsp+0xF0], xmm14
       vmovaps  xmmword ptr [rsp+0xE0], xmm15
       mov      rax, bword ptr [rsp+0x1C0]
       mov      r10, bword ptr [rsp+0x1C8]
       vmovss   xmm0, dword ptr [rsp+0x1D0]
       vmovss   xmm1, dword ptr [rsp+0x1D8]
 
G_M000_IG02:                ;; offset=0x0085
       vxorps   xmm2, xmm2, xmm2
       vmovaps  xmmword ptr [rsp+0xD0], xmm2
       vmovsd   xmm3, qword ptr [r10]
       vinsertps xmm3, xmm3, dword ptr [r10+0x08], 40
       vxorps   xmm4, xmm4, xmm4
       vmaxss   xmm4, xmm4, xmm0
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vbroadcastss xmm4, xmm0
       vxorps   xmm5, xmm5, xmm5
       vcmpltps xmm4, xmm4, xmm5
       vmovss   dword ptr [rsp+0x1D0], xmm0
       vsubss   xmm5, xmm1, xmm0
       vblendvps xmm1, xmm1, xmm5, xmm4
       vmulss   xmm1, xmm1, xmm1
       vmovss   dword ptr [rsp+0xCC], xmm1
       vmovsd   xmm4, qword ptr [r8]
       vinsertps xmm4, xmm4, dword ptr [r8+0x08], 40
       vmovaps  xmm5, xmm4
       vmovsd   xmm6, qword ptr [r9]
       vinsertps xmm6, xmm6, dword ptr [r9+0x08], 40
       movzx    r8, byte  ptr [rdx+0x24]
       movzx    r9, byte  ptr [rdx+0x54]
       and      r8d, r9d
       movzx    r9, byte  ptr [rdx+0x84]
       test     r8d, r9d
       je       G_M000_IG05
 
G_M000_IG03:                ;; offset=0x0117
       vmovups  xmm7, xmmword ptr [rdx]
       vmovups  xmm8, xmmword ptr [rdx+0x30]
       vmovups  xmm9, xmmword ptr [rdx+0x60]
       vsubps   xmm10, xmm8, xmm7
       vsubps   xmm11, xmm7, xmm9
       vsubps   xmm7, xmm4, xmm7
       vsubps   xmm8, xmm4, xmm8
       vsubps   xmm9, xmm4, xmm9
       vpermilps xmm12, xmm10, 9
       vpermilps xmm13, xmm11, 18
       vmulps   xmm12, xmm13, xmm12
       vpermilps xmm10, xmm10, 18
       vpermilps xmm11, xmm11, 9
       vmulps   xmm10, xmm11, xmm10
       vsubps   xmm10, xmm12, xmm10
       vmovaps  xmm11, xmm3
       vsubps   xmm4, xmm4, xmm11
       vpermilps xmm11, xmm10, 9
       vpermilps xmm12, xmm4, 18
       vmulps   xmm11, xmm12, xmm11
       vpermilps xmm10, xmm10, 18
       vpermilps xmm4, xmm4, 9
       vmulps   xmm4, xmm4, xmm10
       vsubps   xmm4, xmm11, xmm4
       vinsertps xmm4, xmm4, xmm4, 56
       vinsertps xmm7, xmm7, xmm7, 56
       vmulps   xmm7, xmm7, xmm4
       vpermilps xmm10, xmm7, -11
       vaddps   xmm10, xmm10, xmm7
       vpermilps xmm7, xmm7, -86
       vaddps   xmm7, xmm7, xmm10
       vinsertps xmm8, xmm8, xmm8, 56
       vmulps   xmm8, xmm8, xmm4
       vpermilps xmm10, xmm8, -11
       vaddps   xmm10, xmm10, xmm8
       vpermilps xmm8, xmm8, -86
       vaddps   xmm8, xmm8, xmm10
       vinsertps xmm9, xmm9, xmm9, 56
       vmulps   xmm4, xmm9, xmm4
       vpermilps xmm9, xmm4, -11
       vaddps   xmm9, xmm9, xmm4
       vpermilps xmm4, xmm4, -86
       vaddps   xmm4, xmm4, xmm9
       vbroadcastss xmm7, xmm7
       vxorps   xmm9, xmm9, xmm9
       vcmpgeps xmm9, xmm7, xmm9
       vbroadcastss xmm8, xmm8
       vxorps   xmm10, xmm10, xmm10
       vcmpgeps xmm10, xmm8, xmm10
       vbroadcastss xmm4, xmm4
       vxorps   xmm11, xmm11, xmm11
       vcmpgeps xmm11, xmm4, xmm11
       vxorps   xmm12, xmm12, xmm12
       vcmpltps xmm8, xmm8, xmm12
       vandps   xmm8, xmm8, xmm9
       vcmpltps xmm4, xmm4, xmm12
       vandps   xmm4, xmm4, xmm10
       vcmpltps xmm7, xmm7, xmm12
       vandps   xmm7, xmm7, xmm11
 
G_M000_IG04:                ;; offset=0x024C
       vorps    xmm9, xmm8, xmm4
       vorps    xmm9, xmm9, xmm7
       vpcmpeqd xmm10, xmm10, xmm10
       vxorps   xmm9, xmm10, xmm9
       vorps    xmm8, xmm9, xmm8
       vandps   xmm9, xmm5, xmm4
       vandnps  xmm10, xmm4, xmmword ptr [rdx]
       vorps    xmm9, xmm10, xmm9
       vmovups  xmmword ptr [rdx], xmm9
       vandps   xmm9, xmm6, xmm4
       vandnps  xmm4, xmm4, xmmword ptr [rdx+0x10]
       vorps    xmm4, xmm4, xmm9
       vmovups  xmmword ptr [rdx+0x10], xmm4
       lea      r8, bword ptr [rdx+0x30]
       vandps   xmm4, xmm5, xmm7
       vandnps  xmm9, xmm7, xmmword ptr [r8]
       vorps    xmm4, xmm9, xmm4
       vmovups  xmmword ptr [r8], xmm4
       vandps   xmm4, xmm6, xmm7
       vandnps  xmm7, xmm7, xmmword ptr [r8+0x10]
       vorps    xmm4, xmm7, xmm4
       vmovups  xmmword ptr [r8+0x10], xmm4
       lea      r8, bword ptr [rdx+0x60]
       vandps   xmm5, xmm5, xmm8
       vandnps  xmm4, xmm8, xmmword ptr [r8]
       vorps    xmm4, xmm4, xmm5
       vmovups  xmmword ptr [r8], xmm4
       vandps   xmm4, xmm6, xmm8
       vandnps  xmm5, xmm8, xmmword ptr [r8+0x10]
       vorps    xmm4, xmm5, xmm4
       vmovups  xmmword ptr [r8+0x10], xmm4
       jmp      G_M000_IG07
 
G_M000_IG05:                ;; offset=0x02E2
       mov      r8d, -1
       xor      r9d, r9d
       cmp      byte  ptr [rdx+0x24], 0
       cmovne   r8d, r9d
       vmovd    xmm4, r8d
       vpbroadcastd xmm4, xmm4
       vandps   xmm7, xmm5, xmm4
       vandnps  xmm8, xmm4, xmmword ptr [rdx]
       vorps    xmm7, xmm8, xmm7
       vmovups  xmmword ptr [rdx], xmm7
       vandps   xmm7, xmm6, xmm4
       vandnps  xmm4, xmm4, xmmword ptr [rdx+0x10]
       vorps    xmm4, xmm4, xmm7
       vmovups  xmmword ptr [rdx+0x10], xmm4
       lea      r8, bword ptr [rdx+0x30]
       mov      r9d, -1
       xor      r11d, r11d
       cmp      byte  ptr [r8+0x24], 0
       cmovne   r9d, r11d
       vmovd    xmm4, r9d
       vpbroadcastd xmm4, xmm4
       vandps   xmm7, xmm5, xmm4
       vandnps  xmm8, xmm4, xmmword ptr [r8]
       vorps    xmm7, xmm8, xmm7
       vmovups  xmmword ptr [r8], xmm7
       vandps   xmm7, xmm6, xmm4
       vandnps  xmm4, xmm4, xmmword ptr [r8+0x10]
       vorps    xmm4, xmm4, xmm7
       vmovups  xmmword ptr [r8+0x10], xmm4
       lea      r8, bword ptr [rdx+0x60]
       mov      r9d, -1
 
G_M000_IG06:                ;; offset=0x036F
       cmp      byte  ptr [r8+0x24], 0
       cmovne   r9d, r11d
       vmovd    xmm4, r9d
       vpbroadcastd xmm4, xmm4
       vandps   xmm5, xmm5, xmm4
       vandnps  xmm7, xmm4, xmmword ptr [r8]
       vorps    xmm5, xmm7, xmm5
       vmovups  xmmword ptr [r8], xmm5
       vandps   xmm5, xmm6, xmm4
       vandnps  xmm4, xmm4, xmmword ptr [r8+0x10]
       vorps    xmm4, xmm4, xmm5
       vmovups  xmmword ptr [r8+0x10], xmm4
 
G_M000_IG07:                ;; offset=0x03A8
       vmovups  xmm4, xmmword ptr [rdx]
       vmovups  xmm5, xmmword ptr [rdx+0x30]
       vmovups  xmm6, xmmword ptr [rdx+0x60]
       vmovaps  xmm7, xmm5
       vsubps   xmm7, xmm7, xmm4
       vsubps   xmm8, xmm4, xmm6
       vmovaps  xmm9, xmm5
       vsubps   xmm9, xmm6, xmm9
       vmovaps  xmmword ptr [rsp+0xB0], xmm9
       vmovaps  xmm10, xmm8
       vmovaps  xmm11, xmm7
       vpermilps xmm12, xmm11, 9
       vpermilps xmm13, xmm10, 18
       vmulps   xmm12, xmm13, xmm12
       vpermilps xmm11, xmm11, 18
       vpermilps xmm10, xmm10, 9
       vmulps   xmm10, xmm10, xmm11
       vsubps   xmm10, xmm12, xmm10
       vinsertps xmm11, xmm10, xmm10, 56
       vdpps    xmm12, xmm11, xmm11, -1
       vsubps   xmm4, xmm4, xmm3
       vsubps   xmm6, xmm6, xmm3
       vmovaps  xmmword ptr [rsp+0xA0], xmm6
       vmovaps  xmm13, xmm4
       vmovaps  xmm14, xmm7
       vpermilps xmm15, xmm14, 9
       vpermilps xmm2, xmm13, 18
       vmulps   xmm2, xmm2, xmm15
       vpermilps xmm14, xmm14, 18
       vpermilps xmm13, xmm13, 9
       vmulps   xmm13, xmm13, xmm14
       vsubps   xmm2, xmm2, xmm13
       vmovaps  xmm13, xmm6
       vmovaps  xmm14, xmm8
       vpermilps xmm15, xmm14, 9
       vpermilps xmm1, xmm13, 18
       vmulps   xmm1, xmm1, xmm15
       vpermilps xmm14, xmm14, 18
       vpermilps xmm13, xmm13, 9
       vmulps   xmm13, xmm13, xmm14
       vsubps   xmm1, xmm1, xmm13
       vinsertps xmm2, xmm2, xmm2, 56
       vmulps   xmm2, xmm2, xmm11
       vpermilps xmm13, xmm2, -11
       vaddps   xmm13, xmm13, xmm2
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm13
       vmovss   dword ptr [rsp+0x9C], xmm2
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm1, xmm1, xmm11
       vpermilps xmm13, xmm1, -11
       vaddps   xmm13, xmm13, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm13
       vmovss   dword ptr [rsp+0x98], xmm1
       vsubss   xmm13, xmm12, xmm1
       vsubss   xmm13, xmm13, xmm2
       vmovss   dword ptr [rsp+0x94], xmm13
       vxorps   xmm14, xmm14, xmm14
       vucomiss xmm14, xmm13
       seta     r8b
 
G_M000_IG08:                ;; offset=0x04F1
       movzx    r8, r8b
       vucomiss xmm14, xmm1
       seta     r9b
       movzx    r9, r9b
       vinsertps xmm14, xmm7, xmm7, 56
       vdpps    xmm15, xmm14, xmm14, -1
       vmovss   dword ptr [rsp+0x90], xmm15
       vmovaps  xmm13, xmm9
       vinsertps xmm13, xmm13, xmm13, 56
       vdpps    xmm9, xmm13, xmm13, -1
       vinsertps xmm15, xmm8, xmm8, 56
       vmovaps  xmmword ptr [rsp], xmm15
       vdpps    xmm0, xmm15, xmm15, -1
       vmovss   xmm1, dword ptr [rsp+0x90]
       vmaxss   xmm1, xmm1, xmm9
       vmaxss   xmm1, xmm1, xmm0
       vmovss   dword ptr [rsp+0x8C], xmm1
       vmulss   xmm1, xmm1, dword ptr [reloc @RWD00]
       vucomiss xmm1, xmm12
       setae    r11b
       movzx    r11, r11b
       vmovss   xmm1, dword ptr [reloc @RWD04]
       vucomiss xmm1, dword ptr [rsp+0x8C]
       seta     bl
       movzx    rbx, bl
       test     ebx, ebx
       sete     sil
       movzx    rsi, sil
       and      esi, r11d
       vmovsd   xmm1, qword ptr [r10]
       vinsertps xmm1, xmm1, dword ptr [r10+0x08], 40
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm1, xmm1, xmm11
       vpermilps xmm11, xmm1, -11
       vaddps   xmm11, xmm11, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm11
       vbroadcastss xmm1, xmm1
       vxorps   xmm11, xmm11, xmm11
       vcmpltps xmm1, xmm1, xmm11
       vxorps   xmm11, xmm10, xmmword ptr [reloc @RWD16]
       vandps   xmm11, xmm11, xmm1
       vandnps  xmm1, xmm1, xmm10
       vorps    xmm10, xmm1, xmm11
       vxorps   xmm1, xmm1, xmm1
       vucomiss xmm1, xmm2
       seta     r10b
       movzx    r10, r10b
       or       r8d, r10d
       or       r8d, r9d
       movzx    r8, r8b
       vxorps   xmm1, xmm4, xmmword ptr [reloc @RWD16]
       mov      r10d, 1
       vmovss   xmm11, dword ptr [reloc @RWD32]
       vmovss   dword ptr [rdx+0x20], xmm11
       xor      r9d, r9d
       mov      dword ptr [rdx+0x50], r9d
 
G_M000_IG09:                ;; offset=0x061A
       mov      dword ptr [rdx+0x80], r9d
       vmovss   dword ptr [rdx+0x90], xmm11
       vinsertps xmm2, xmm4, xmm4, 56
       vdpps    xmm15, xmm2, xmm2, -1
       vmovss   xmm6, dword ptr [rsp+0xCC]
       vucomiss xmm6, xmm15
       seta     r9b
       movzx    r9, r9b
       test     r9d, ebx
       je       SHORT G_M000_IG11
 
G_M000_IG10:                ;; offset=0x0650
       mov      byte  ptr [rax], 1
 
G_M000_IG11:                ;; offset=0x0653
       or       esi, r8d
       cmp      byte  ptr [rax], 0
       sete     r9b
       movzx    r9, r9b
       test     esi, r9d
       je       G_M000_IG16
 
G_M000_IG12:                ;; offset=0x066A
       vmovss   xmm15, dword ptr [rsp+0x90]
       vdivss   xmm1, xmm11, xmm15
       vmovss   dword ptr [rsp+0x88], xmm1
       vdivss   xmm1, xmm11, xmm9
       vmovss   dword ptr [rsp+0x84], xmm1
       vdivss   xmm1, xmm11, xmm0
       vmovss   dword ptr [rsp+0x80], xmm1
       vsubps   xmm5, xmm5, xmm3
       vmulps   xmm14, xmm14, xmm2
       vpermilps xmm1, xmm14, -11
       vaddps   xmm1, xmm1, xmm14
       vpermilps xmm14, xmm14, -86
       vaddps   xmm1, xmm14, xmm1
       vmovss   dword ptr [rsp+0x7C], xmm1
       vinsertps xmm14, xmm5, xmm5, 56
       vmulps   xmm13, xmm14, xmm13
       vpermilps xmm14, xmm13, -11
       vaddps   xmm14, xmm14, xmm13
       vpermilps xmm13, xmm13, -86
       vaddps   xmm13, xmm13, xmm14
       vmovss   dword ptr [rsp+0x78], xmm13
       vmovaps  xmm14, xmmword ptr [rsp+0xA0]
       vmovaps  xmm13, xmm14
       vinsertps xmm13, xmm13, xmm13, 56
       vmulps   xmm13, xmm13, xmmword ptr [rsp]
       vpermilps xmm1, xmm13, -11
       vaddps   xmm1, xmm1, xmm13
       vpermilps xmm13, xmm13, -86
       vaddps   xmm1, xmm13, xmm1
       vmovss   dword ptr [rsp+0x74], xmm1
       vmovss   xmm1, dword ptr [rsp+0x7C]
       vxorps   xmm1, xmm1, xmmword ptr [reloc @RWD16]
       vminss   xmm1, xmm15, xmm1
       vxorps   xmm13, xmm13, xmm13
       vmaxss   xmm1, xmm13, xmm1
       vmovss   xmm15, dword ptr [rsp+0x78]
       vxorps   xmm13, xmm15, xmmword ptr [reloc @RWD16]
       vmovaps  xmm15, xmm9
       vminss   xmm13, xmm15, xmm13
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm13, xmm15, xmm13
       vmulss   xmm1, xmm1, dword ptr [rsp+0x88]
       vmovss   dword ptr [rsp+0x70], xmm1
       vmulss   xmm13, xmm13, dword ptr [rsp+0x84]
       vmovss   dword ptr [rsp+0x6C], xmm13
       vmovss   xmm6, dword ptr [rsp+0x74]
       vxorps   xmm6, xmm6, xmmword ptr [reloc @RWD16]
       vmovaps  xmm15, xmm0
       vminss   xmm6, xmm15, xmm6
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm6, xmm15, xmm6
       vmulss   xmm6, xmm6, dword ptr [rsp+0x80]
       vmovss   dword ptr [rsp+0x68], xmm6
       vbroadcastss xmm15, xmm1
       vmulps   xmm15, xmm15, xmm7
       vmovaps  xmmword ptr [rsp+0x50], xmm15
       vmovaps  xmm15, xmmword ptr [rsp+0xB0]
       vbroadcastss xmm13, xmm13
       vmulps   xmm13, xmm13, xmm15
       vbroadcastss xmm15, xmm6
 
G_M000_IG13:                ;; offset=0x07CA
       vmulps   xmm15, xmm15, xmm8
       vmovaps  xmmword ptr [rsp+0x40], xmm15
       vmovaps  xmm15, xmmword ptr [rsp+0x50]
       vaddps   xmm15, xmm15, xmm4
       vmovaps  xmmword ptr [rsp+0x30], xmm15
       vaddps   xmm13, xmm13, xmm5
       vmovaps  xmmword ptr [rsp+0x20], xmm13
       vmovaps  xmm13, xmm14
       vmovaps  xmm15, xmmword ptr [rsp+0x40]
       vaddps   xmm13, xmm15, xmm13
       vmovaps  xmm15, xmmword ptr [rsp+0x30]
       vinsertps xmm15, xmm15, xmm15, 56
       vdpps    xmm15, xmm15, xmm15, -1
       vmovss   dword ptr [rsp+0x1C], xmm15
       vmovaps  xmm6, xmmword ptr [rsp+0x20]
       vinsertps xmm6, xmm6, xmm6, 56
       vdpps    xmm6, xmm6, xmm6, -1
       vmovss   dword ptr [rsp+0x18], xmm6
       vinsertps xmm13, xmm13, xmm13, 56
       vdpps    xmm13, xmm13, xmm13, -1
       vbroadcastss xmm9, xmm9
       vxorps   xmm1, xmm1, xmm1
       vcmpeqps xmm1, xmm1, xmm9
       vbroadcastss xmm0, xmm0
       vxorps   xmm9, xmm9, xmm9
       vcmpeqps xmm0, xmm9, xmm0
       vbroadcastss xmm9, xmm15
       vbroadcastss xmm6, xmm6
       vcmpltps xmm15, xmm9, xmm6
       vorps    xmm1, xmm15, xmm1
       vbroadcastss xmm15, xmm13
       vcmpltps xmm9, xmm9, xmm15
       vorps    xmm9, xmm9, xmm0
       vcmpltps xmm6, xmm6, xmm15
       vandps   xmm1, xmm1, xmm9
       vorps    xmm0, xmm6, xmm0
       vandnps  xmm0, xmm1, xmm0
       vmovss   xmm6, dword ptr [rsp+0x1C]
       vandps   xmm6, xmm6, xmm1
       vmovss   xmm9, dword ptr [rsp+0x18]
       vandps   xmm9, xmm9, xmm0
       vandnps  xmm13, xmm0, xmm13
       vorps    xmm9, xmm13, xmm9
       vandnps  xmm9, xmm1, xmm9
       vorps    xmm6, xmm9, xmm6
       vmovss   xmm9, dword ptr [rsp+0xCC]
       vucomiss xmm9, xmm6
       jb       SHORT G_M000_IG14
       mov      byte  ptr [rax], 1
 
G_M000_IG14:                ;; offset=0x08C7
       vmovss   xmm6, dword ptr [rsp+0x70]
       vandps   xmm6, xmm6, xmm1
       vmovss   xmm13, dword ptr [rsp+0x6C]
       vandps   xmm13, xmm13, xmm0
       vmovss   xmm15, dword ptr [rsp+0x68]
       vandnps  xmm15, xmm0, xmm15
       vorps    xmm13, xmm15, xmm13
       vandnps  xmm13, xmm1, xmm13
       vorps    xmm6, xmm13, xmm6
       vmovaps  xmm15, xmmword ptr [rsp+0xB0]
       vandps   xmm13, xmm15, xmm0
       vandps   xmm7, xmm7, xmm1
       vandnps  xmm8, xmm1, xmm8
       vorps    xmm7, xmm8, xmm7
       vandnps  xmm7, xmm0, xmm7
       vorps    xmm7, xmm7, xmm13
       vxorps   xmm8, xmm6, xmmword ptr [reloc @RWD16]
       vbroadcastss xmm8, xmm8
       vmulps   xmm7, xmm8, xmm7
       vandps   xmm5, xmm5, xmm0
       vandps   xmm4, xmm4, xmm1
       vandnps  xmm8, xmm1, xmm14
       vorps    xmm4, xmm8, xmm4
       vandnps  xmm4, xmm0, xmm4
       vorps    xmm4, xmm4, xmm5
       vsubps   xmm4, xmm7, xmm4
       vbroadcastss xmm5, xmm6
       vxorps   xmm7, xmm7, xmm7
       vcmpeqps xmm5, xmm7, xmm5
       vbroadcastss xmm7, xmm6
       vcmpeqps xmm7, xmm7, xmmword ptr [reloc @RWD48]
       vpand    xmm8, xmm5, xmmword ptr [reloc @RWD64]
       vpand    xmm13, xmm7, xmmword ptr [reloc @RWD80]
       vpandn   xmm14, xmm7, xmmword ptr [reloc @RWD96]
       vpor     xmm13, xmm14, xmm13
       vmovd    r10d, xmm13
       vmovd    xmm13, r10d
       vpandn   xmm13, xmm5, xmm13
       vpor     xmm8, xmm13, xmm8
       vmovd    r10d, xmm8
       vpand    xmm8, xmm5, xmmword ptr [reloc @RWD80]
       vpand    xmm13, xmm7, xmmword ptr [reloc @RWD112]
       vpandn   xmm14, xmm7, xmmword ptr [reloc @RWD128]
       vpor     xmm13, xmm14, xmm13
       vmovd    r9d, xmm13
       vmovd    xmm13, r9d
       vpandn   xmm13, xmm5, xmm13
       vpor     xmm8, xmm13, xmm8
       vmovd    r9d, xmm8
       vmovd    xmm8, r9d
       vpand    xmm8, xmm8, xmm0
       vpand    xmm13, xmm5, xmmword ptr [reloc @RWD112]
       vpand    xmm14, xmm7, xmmword ptr [reloc @RWD64]
       vpandn   xmm7, xmm7, xmmword ptr [reloc @RWD144]
       vpor     xmm7, xmm7, xmm14
       vmovd    r9d, xmm7
       vmovd    xmm7, r9d
       vpandn   xmm5, xmm5, xmm7
       vpor     xmm5, xmm5, xmm13
       vmovd    r9d, xmm5
       vmovd    xmm5, r9d
       vpandn   xmm5, xmm0, xmm5
       vpor     xmm5, xmm5, xmm8
       vmovd    r9d, xmm5
 
G_M000_IG15:                ;; offset=0x0A1E
       vmovd    xmm5, r10d
       vpand    xmm5, xmm5, xmm1
       vmovd    xmm7, r9d
       vpandn   xmm7, xmm1, xmm7
       vpor     xmm5, xmm7, xmm5
       vmovd    r10d, xmm5
       vsubss   xmm5, xmm11, xmm6
       vandps   xmm7, xmm5, xmm1
       vxorps   xmm8, xmm8, xmm8
       vandps   xmm8, xmm8, xmm0
       vmovaps  xmm13, xmm6
       vandnps  xmm13, xmm0, xmm13
       vorps    xmm8, xmm13, xmm8
       vandnps  xmm8, xmm1, xmm8
       vorps    xmm7, xmm8, xmm7
       vmovss   dword ptr [rdx+0x20], xmm7
       vmovaps  xmm7, xmm6
       vandps   xmm7, xmm7, xmm1
       vandps   xmm8, xmm5, xmm0
       vxorps   xmm13, xmm13, xmm13
       vandnps  xmm13, xmm0, xmm13
       vorps    xmm8, xmm13, xmm8
       vandnps  xmm8, xmm1, xmm8
       vorps    xmm7, xmm8, xmm7
       vmovss   dword ptr [rdx+0x50], xmm7
       vxorps   xmm7, xmm7, xmm7
       vandps   xmm7, xmm7, xmm1
       vandps   xmm6, xmm6, xmm0
       vandnps  xmm0, xmm0, xmm5
       vorps    xmm0, xmm0, xmm6
       vandnps  xmm0, xmm1, xmm0
       vorps    xmm0, xmm0, xmm7
       vmovss   dword ptr [rdx+0x80], xmm0
       vmovaps  xmm1, xmm4
 
G_M000_IG16:                ;; offset=0x0AB7
       test     r8d, r8d
       sete     r8b
       movzx    r8, r8b
       test     r11d, r11d
       sete     r9b
       movzx    r9, r9b
       and      r8d, r9d
       cmp      byte  ptr [rax], 0
       sete     r9b
       movzx    r9, r9b
       and      r8d, r9d
       je       SHORT G_M000_IG19
 
G_M000_IG17:                ;; offset=0x0AE0
       vmovaps  xmm1, xmm10
       vinsertps xmm0, xmm1, xmm1, 56
       vmulps   xmm0, xmm0, xmm2
       vpermilps xmm1, xmm0, -11
       vaddps   xmm1, xmm1, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm1
       vmulss   xmm0, xmm0, xmm0
       vmulss   xmm1, xmm12, dword ptr [rsp+0xCC]
       vucomiss xmm1, xmm0
       jbe      SHORT G_M000_IG18
       mov      byte  ptr [rax], 1
 
G_M000_IG18:                ;; offset=0x0B19
       vmovaps  xmm1, xmm10
       mov      r10d, 7
       vmovss   xmm13, dword ptr [rsp+0x94]
       vmovss   dword ptr [rdx+0x20], xmm13
       vmovss   xmm0, dword ptr [rsp+0x98]
       vmovss   dword ptr [rdx+0x50], xmm0
       vmovss   xmm2, dword ptr [rsp+0x9C]
       vmovss   dword ptr [rdx+0x80], xmm2
       vmovss   dword ptr [rdx+0x90], xmm12
 
G_M000_IG19:                ;; offset=0x0B59
       mov      r9d, r10d
       and      r9d, 1
       setg     r9b
       mov      byte  ptr [rdx+0x24], r9b
       mov      r9d, r10d
       and      r9d, 2
       setg     r9b
       mov      byte  ptr [rdx+0x54], r9b
       and      r10d, 4
       setg     r10b
       mov      byte  ptr [rdx+0x84], r10b
       cmp      byte  ptr [rax], 0
       jne      G_M000_IG21
 
G_M000_IG20:                ;; offset=0x0B8F
       vmovaps  xmm0, xmm1
       vmulps   xmm0, xmm0, xmmword ptr [reloc @RWD160]
       vaddps   xmm0, xmm0, xmm3
       vbroadcastss xmm2, dword ptr [rsp+0x1D0]
       vxorps   xmm3, xmm3, xmm3
       vcmpleps xmm2, xmm2, xmm3
       mov      eax, -1
       xor      edx, edx
       test     r8d, r8d
       cmove    eax, edx
       vmovd    xmm3, eax
       vpbroadcastd xmm3, xmm3
       vorps    xmm2, xmm3, xmm2
       vandps   xmm1, xmm1, xmm2
       vandnps  xmm0, xmm2, xmm0
       vorps    xmm1, xmm0, xmm1
       vmovaps  xmm0, xmm1
       vinsertps xmm0, xmm0, xmm0, 56
       vdpps    xmm0, xmm0, xmm0, -1
       vsqrtss  xmm0, xmm0, xmm0
       vdivss   xmm0, xmm11, xmm0
       vbroadcastss xmm0, xmm0
       vmulps   xmm2, xmm0, xmm1
       vmovsd   qword ptr [rsp+0xD0], xmm2
       vextractps dword ptr [rsp+0xD8], xmm2, 2
 
G_M000_IG21:                ;; offset=0x0C0D
       vmovaps  xmm2, xmmword ptr [rsp+0xD0]
       vmovsd   qword ptr [rcx], xmm2
       vextractps dword ptr [rcx+0x08], xmm2, 2
       mov      rax, rcx
 
G_M000_IG22:                ;; offset=0x0C24
       vmovaps  xmm6, xmmword ptr [rsp+0x170]
       vmovaps  xmm7, xmmword ptr [rsp+0x160]
       vmovaps  xmm8, xmmword ptr [rsp+0x150]
       vmovaps  xmm9, xmmword ptr [rsp+0x140]
       vmovaps  xmm10, xmmword ptr [rsp+0x130]
       vmovaps  xmm11, xmmword ptr [rsp+0x120]
       vmovaps  xmm12, xmmword ptr [rsp+0x110]
       vmovaps  xmm13, xmmword ptr [rsp+0x100]
       vmovaps  xmm14, xmmword ptr [rsp+0xF0]
       vmovaps  xmm15, xmmword ptr [rsp+0xE0]
       add      rsp, 392
       pop      rbx
       pop      rsi
       ret      
 
RWD00  	dd	2EDBE6FFh		;     1e-10
RWD04  	dd	283424DCh		;     1e-14
RWD08  	dd	00000000h, 00000000h
RWD16  	dq	8000000080000000h, 8000000080000000h
RWD32  	dd	3F800000h		;         1
RWD36  	dd	00000000h, 00000000h, 00000000h
RWD48  	dq	3F8000003F800000h, 3F8000003F800000h
RWD64  	dq	0000000100000001h, 0000000100000001h
RWD80  	dq	0000000200000002h, 0000000200000002h
RWD96  	dq	0000000300000003h, 0000000300000003h
RWD112 	dq	0000000400000004h, 0000000400000004h
RWD128 	dq	0000000600000006h, 0000000600000006h
RWD144 	dq	0000000500000005h, 0000000500000005h
RWD160 	dq	4080000040800000h, 4080000040800000h

; Total bytes of code 3208

