; Assembly listing for method BepuPhysics.CollisionDetection.CollisionTasks.CylinderPairTester:Test(byref,byref,byref,byref,byref,byref,int,byref) (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rbp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 203 single block inlinees; 1 inlinees without PGO data

G_M000_IG01:                ;; offset=0x0000
       push     rbp
       push     r14
       push     rdi
       push     rsi
       push     rbx
       lea      r11, [rsp-0x3390]
       call     CORINFO_HELP_STACK_PROBE
       mov      rsp, r11
       vmovaps  xmmword ptr [rsp+0x3380], xmm6
       vmovaps  xmmword ptr [rsp+0x3370], xmm7
       vmovaps  xmmword ptr [rsp+0x3360], xmm8
       vmovaps  xmmword ptr [rsp+0x3350], xmm9
       vmovaps  xmmword ptr [rsp+0x3340], xmm10
       vmovaps  xmmword ptr [rsp+0x3330], xmm11
       vmovaps  xmmword ptr [rsp+0x3320], xmm12
       vmovaps  xmmword ptr [rsp+0x3310], xmm13
       vmovaps  xmmword ptr [rsp+0x3300], xmm14
       vmovaps  xmmword ptr [rsp+0x32F0], xmm15
       lea      rbp, [rsp+0x33B0]
       mov      rsi, rcx
       mov      rdi, rdx
       mov      r14, r8
       mov      r8, bword ptr [rbp+0x30]
       mov      rcx, bword ptr [rbp+0x38]
       mov      rbx, bword ptr [rbp+0x48]
 
G_M000_IG02:                ;; offset=0x008D
       vmovups  ymm0, ymmword ptr [r8]
       vaddps   ymm1, ymm0, ymm0
       vmovups  ymm2, ymmword ptr [r8+0x20]
       vaddps   ymm3, ymm2, ymm2
       vmovups  ymm4, ymmword ptr [r8+0x40]
       vaddps   ymm5, ymm4, ymm4
       vmulps   ymm6, ymm2, ymm3
       vmulps   ymm7, ymm4, ymm5
       vbroadcastss ymm8, dword ptr [reloc @RWD00]
       vsubps   ymm9, ymm8, ymm6
       vsubps   ymm9, ymm9, ymm7
       vmulps   ymm2, ymm2, ymm1
       vmovups  ymm10, ymmword ptr [r8+0x60]
       vmulps   ymm5, ymm10, ymm5
       vaddps   ymm11, ymm2, ymm5
       vmulps   ymm12, ymm4, ymm1
       vmulps   ymm13, ymm10, ymm3
       vsubps   ymm14, ymm12, ymm13
       vmulps   ymm0, ymm0, ymm1
       vsubps   ymm2, ymm2, ymm5
       vsubps   ymm0, ymm8, ymm0
       vsubps   ymm5, ymm0, ymm7
       vmovups  ymmword ptr [rbp-0x2D70], ymm5
       vmulps   ymm1, ymm10, ymm1
       vmulps   ymm3, ymm4, ymm3
       vaddps   ymm4, ymm3, ymm1
       vmovups  ymmword ptr [rbp-0x2D90], ymm4
       vaddps   ymm7, ymm12, ymm13
       vmovups  ymmword ptr [rbp-0x2DB0], ymm7
       vsubps   ymm1, ymm3, ymm1
       vmovups  ymmword ptr [rbp-0x2DD0], ymm1
       vsubps   ymm0, ymm0, ymm6
       vmovups  ymmword ptr [rbp-0x2DF0], ymm0
       vmovups  ymm3, ymmword ptr [rcx]
       vaddps   ymm6, ymm3, ymm3
       vmovups  ymm10, ymmword ptr [rcx+0x20]
       vaddps   ymm12, ymm10, ymm10
       vmovups  ymm13, ymmword ptr [rcx+0x40]
       vaddps   ymm15, ymm13, ymm13
       vmovups  ymmword ptr [rbp-0x9F0], ymm15
       vmulps   ymm15, ymm10, ymm12
       vmovups  ymmword ptr [rbp-0xA10], ymm15
       vmulps   ymm15, ymm13, ymmword ptr [rbp-0x9F0]
       vmovups  ymmword ptr [rbp-0xA30], ymm15
       vsubps   ymm15, ymm8, ymmword ptr [rbp-0xA10]
       vsubps   ymm15, ymm15, ymmword ptr [rbp-0xA30]
       vmulps   ymm10, ymm10, ymm6
       vmovups  ymmword ptr [rbp-0xA50], ymm10
       vmovups  ymm10, ymmword ptr [rcx+0x60]
       vmulps   ymm0, ymm10, ymmword ptr [rbp-0x9F0]
       vmovups  ymmword ptr [rbp-0xA70], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0xA50]
       vaddps   ymm0, ymm0, ymmword ptr [rbp-0xA70]
       vmovups  ymmword ptr [rbp-0x2E10], ymm0
       vmulps   ymm1, ymm13, ymm6
       vmovups  ymmword ptr [rbp-0xA90], ymm1
       vmulps   ymm1, ymm10, ymm12
       vmovups  ymmword ptr [rbp-0xAB0], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0xA90]
 
G_M000_IG03:                ;; offset=0x01DE
       vsubps   ymm1, ymm1, ymmword ptr [rbp-0xAB0]
       vmovups  ymmword ptr [rbp-0x2E30], ymm1
       vmulps   ymm3, ymm3, ymm6
       vmovups  ymmword ptr [rbp-0xAD0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0xA50]
       vsubps   ymm3, ymm3, ymmword ptr [rbp-0xA70]
       vmovups  ymmword ptr [rbp-0x2E50], ymm3
       vsubps   ymm7, ymm8, ymmword ptr [rbp-0xAD0]
       vmovups  ymmword ptr [rbp-0x2F70], ymm7
       vsubps   ymm7, ymm7, ymmword ptr [rbp-0xA30]
       vmulps   ymm6, ymm10, ymm6
       vmulps   ymm10, ymm13, ymm12
       vaddps   ymm12, ymm10, ymm6
       vmovups  ymm13, ymmword ptr [rbp-0xA90]
       vaddps   ymm13, ymm13, ymmword ptr [rbp-0xAB0]
       vsubps   ymm6, ymm10, ymm6
       vmovups  ymm10, ymmword ptr [rbp-0x2F70]
       vsubps   ymm10, ymm10, ymmword ptr [rbp-0xA10]
       vmulps   ymm4, ymm15, ymm9
       vmulps   ymm5, ymm0, ymm11
       vaddps   ymm4, ymm5, ymm4
       vmulps   ymm5, ymm1, ymm14
       vaddps   ymm4, ymm5, ymm4
       vmovups  ymmword ptr [rbp-0x1E0], ymm4
       vmulps   ymm4, ymm3, ymm9
       vmulps   ymm5, ymm7, ymm11
       vaddps   ymm4, ymm5, ymm4
       vmulps   ymm5, ymm12, ymm14
       vaddps   ymm4, ymm5, ymm4
       vmovups  ymmword ptr [rbp-0x1C0], ymm4
       vmulps   ymm4, ymm13, ymm9
       vmulps   ymm5, ymm6, ymm11
       vaddps   ymm4, ymm5, ymm4
       vmulps   ymm5, ymm10, ymm14
       vaddps   ymm4, ymm5, ymm4
       vmovups  ymmword ptr [rbp-0x1A0], ymm4
       vmulps   ymm4, ymm15, ymm2
       vmovups  ymm5, ymmword ptr [rbp-0x2D70]
       vmulps   ymm9, ymm0, ymm5
       vaddps   ymm4, ymm9, ymm4
       vmovups  ymm9, ymmword ptr [rbp-0x2D90]
       vmulps   ymm11, ymm1, ymm9
       vaddps   ymm4, ymm11, ymm4
       vmovups  ymmword ptr [rbp-0x180], ymm4
       vmulps   ymm4, ymm3, ymm2
       vmulps   ymm11, ymm7, ymm5
       vaddps   ymm4, ymm11, ymm4
       vmulps   ymm11, ymm12, ymm9
       vaddps   ymm4, ymm11, ymm4
       vmovups  ymmword ptr [rbp-0x160], ymm4
       vmulps   ymm2, ymm13, ymm2
       vmulps   ymm4, ymm6, ymm5
       vaddps   ymm2, ymm4, ymm2
       vmulps   ymm4, ymm10, ymm9
       vaddps   ymm2, ymm4, ymm2
       vmovups  ymmword ptr [rbp-0x140], ymm2
       vmovups  ymm2, ymmword ptr [rbp-0x2DB0]
       vmulps   ymm4, ymm15, ymm2
       vmovups  ymm5, ymmword ptr [rbp-0x2DD0]
 
G_M000_IG04:                ;; offset=0x0333
       vmulps   ymm9, ymm0, ymm5
       vaddps   ymm4, ymm9, ymm4
       vmovups  ymm9, ymmword ptr [rbp-0x2DF0]
       vmulps   ymm11, ymm1, ymm9
       vaddps   ymm4, ymm11, ymm4
       vmovups  ymmword ptr [rbp-0x120], ymm4
       vmulps   ymm4, ymm3, ymm2
       vmulps   ymm11, ymm7, ymm5
       vaddps   ymm4, ymm11, ymm4
       vmulps   ymm11, ymm12, ymm9
       vaddps   ymm4, ymm11, ymm4
       vmovups  ymmword ptr [rbp-0x100], ymm4
       vmulps   ymm2, ymm13, ymm2
       vmulps   ymm4, ymm6, ymm5
       vaddps   ymm2, ymm4, ymm2
       vmulps   ymm4, ymm10, ymm9
       vaddps   ymm2, ymm4, ymm2
       vmovups  ymmword ptr [rbp-0xE0], ymm2
       vmovups  ymm2, ymmword ptr [r9]
       vmulps   ymm4, ymm15, ymm2
       vmovups  ymm5, ymmword ptr [r9+0x20]
       vmulps   ymm9, ymm0, ymm5
       vaddps   ymm4, ymm9, ymm4
       vmovups  ymm9, ymmword ptr [r9+0x40]
       vmulps   ymm11, ymm1, ymm9
       vaddps   ymm11, ymm11, ymm4
       vmovups  ymmword ptr [rbp-0x1470], ymm11
       vmulps   ymm4, ymm3, ymm2
       vmulps   ymm14, ymm7, ymm5
       vaddps   ymm4, ymm14, ymm4
       vmulps   ymm14, ymm12, ymm9
       vaddps   ymm14, ymm14, ymm4
       vmovups  ymmword ptr [rbp-0x1490], ymm14
       vmulps   ymm2, ymm13, ymm2
       vmulps   ymm4, ymm6, ymm5
       vaddps   ymm2, ymm4, ymm2
       vmulps   ymm4, ymm10, ymm9
       vaddps   ymm9, ymm4, ymm2
       vmovups  ymmword ptr [rbp-0x14B0], ymm9
       vbroadcastss ymm2, dword ptr [reloc @RWD04]
       vmovups  ymmword ptr [rbp-0x3050], ymm2
       vxorps   ymm4, ymm2, ymm11
       vmovups  ymmword ptr [rbp-0x240], ymm4
       vxorps   ymm4, ymm2, ymm14
       vmovups  ymmword ptr [rbp-0x220], ymm4
       vxorps   ymm4, ymm2, ymm9
       vmovups  ymmword ptr [rbp-0x200], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x240]
       vmulps   ymm4, ymm4, ymmword ptr [rbp-0x240]
       vmovups  ymm5, ymmword ptr [rbp-0x220]
       vmulps   ymm5, ymm5, ymmword ptr [rbp-0x220]
       vaddps   ymm4, ymm5, ymm4
       vmovups  ymm5, ymmword ptr [rbp-0x200]
       vmulps   ymm5, ymm5, ymmword ptr [rbp-0x200]
       vaddps   ymm4, ymm5, ymm4
       vsqrtps  ymm4, ymm4
       vdivps   ymm5, ymm8, ymm4
       vmovups  ymmword ptr [rbp-0x570], ymm5
       vmovups  ymm5, ymmword ptr [rbp-0x240]
 
G_M000_IG05:                ;; offset=0x047E
       vmulps   ymm5, ymm5, ymmword ptr [rbp-0x570]
       vmovups  ymmword ptr [rbp-0x2A0], ymm5
       vmovups  ymm5, ymmword ptr [rbp-0x220]
       vmulps   ymm5, ymm5, ymmword ptr [rbp-0x570]
       vmovups  ymmword ptr [rbp-0x280], ymm5
       vmovups  ymm5, ymmword ptr [rbp-0x200]
       vmulps   ymm5, ymm5, ymmword ptr [rbp-0x570]
       vmovups  ymmword ptr [rbp-0x260], ymm5
       vcmpltps ymm4, ymm4, ymmword ptr [reloc @RWD32]
       vxorps   ymm5, ymm5, ymm5
       vandps   ymm5, ymm5, ymm4
       vandnps  ymm9, ymm4, ymmword ptr [rbp-0x2A0]
       vorps    ymm5, ymm9, ymm5
       vmovups  ymmword ptr [rbp-0x2A0], ymm5
       vandps   ymm5, ymm8, ymm4
       vandnps  ymm9, ymm4, ymmword ptr [rbp-0x280]
       vorps    ymm5, ymm9, ymm5
       vmovups  ymmword ptr [rbp-0x280], ymm5
       vxorps   ymm5, ymm5, ymm5
       vandps   ymm5, ymm5, ymm4
       vandnps  ymm4, ymm4, ymmword ptr [rbp-0x260]
       vorps    ymm4, ymm4, ymm5
       vmovups  ymmword ptr [rbp-0x260], ymm4
       vxorps   xmm4, xmm4, xmm4
       vcvtsi2ss xmm4, xmm4, dword ptr [rbp+0x40]
       vbroadcastss ymm4, ymm4
       vcmpleps ymm4, ymm4, ymmword ptr [reloc @RWD64]
       vmovups  ymmword ptr [rbp-0x2D0], ymm4
       vxorps   ymm4, ymm2, ymmword ptr [r14]
       vmovups  ymmword ptr [rbp-0x2F0], ymm4
       vmovups  ymm4, ymmword ptr [rsi+0x20]
       vmovups  ymm5, ymmword ptr [rsi]
       vcmpeqps ymm9, ymm4, ymm5
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymm5
       vandps   ymm9, ymm14, ymm9
       vcmpneqps ymm14, ymm4, ymm4
       vorps    ymm9, ymm14, ymm9
       vcmpltps ymm14, ymm5, ymm4
       vorps    ymm9, ymm14, ymm9
       vblendvps ymm4, ymm5, ymm4, ymm9
       vmovups  ymm5, ymmword ptr [rdi+0x20]
       vmovups  ymm9, ymmword ptr [rdi]
       vcmpeqps ymm14, ymm5, ymm9
       vxorps   ymm11, ymm11, ymm11
       vpcmpgtd ymm11, ymm11, ymm9
       vandps   ymm11, ymm11, ymm14
       vcmpneqps ymm14, ymm5, ymm5
       vorps    ymm11, ymm14, ymm11
       vcmpltps ymm14, ymm9, ymm5
       vorps    ymm11, ymm14, ymm11
       vblendvps ymm5, ymm9, ymm5, ymm11
       vcmpeqps ymm9, ymm4, ymm5
       vxorps   ymm11, ymm11, ymm11
       vpcmpgtd ymm11, ymm11, ymm4
       vandps   ymm9, ymm11, ymm9
       vcmpneqps ymm11, ymm4, ymm4
       vorps    ymm9, ymm11, ymm9
       vcmpltps ymm11, ymm4, ymm5
 
G_M000_IG06:                ;; offset=0x05D3
       vorps    ymm9, ymm11, ymm9
       vblendvps ymm4, ymm5, ymm4, ymm9
       vmulps   ymm4, ymm4, ymmword ptr [reloc @RWD96]
       vmovups  ymmword ptr [rbp-0x570], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x280]
       vxorps   ymm5, ymm5, ymm5
       vcmpgtps ymm4, ymm4, ymm5
       vmovups  ymm5, ymmword ptr [rdi+0x20]
       vxorps   ymm9, ymm2, ymm5
       vblendvps ymm4, ymm9, ymm5, ymm4
       vmovups  ymmword ptr [rbp-0x23D0], ymm4
       vmovups  ymm5, ymmword ptr [rbp-0x2A0]
       vmulps   ymm5, ymm5, ymmword ptr [rbp-0x2A0]
       vmovups  ymm9, ymmword ptr [rbp-0x260]
       vmulps   ymm9, ymm9, ymmword ptr [rbp-0x260]
       vaddps   ymm5, ymm9, ymm5
       vsqrtps  ymm5, ymm5
       vmovups  ymm9, ymmword ptr [rdi]
       vdivps   ymm9, ymm9, ymm5
       vcmpgtps ymm5, ymm5, ymmword ptr [reloc @RWD128]
       vmulps   ymm11, ymm9, ymmword ptr [rbp-0x2A0]
       vandps   ymm11, ymm11, ymm5
       vxorps   ymm14, ymm14, ymm14
       vandnps  ymm14, ymm5, ymm14
       vorps    ymm11, ymm14, ymm11
       vmulps   ymm9, ymm9, ymmword ptr [rbp-0x260]
       vandps   ymm9, ymm9, ymm5
       vxorps   ymm14, ymm14, ymm14
       vandnps  ymm5, ymm5, ymm14
       vorps    ymm5, ymm5, ymm9
       vmovups  ymmword ptr [rbp-0x23F0], ymm5
       vxorps   ymm9, ymm2, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x2410], ymm9
       vxorps   ymm14, ymm2, ymmword ptr [rbp-0x280]
       vmovups  ymmword ptr [rbp-0x2430], ymm14
       vxorps   ymm14, ymm2, ymmword ptr [rbp-0x260]
       vmovups  ymmword ptr [rbp-0x2450], ymm14
       vmulps   ymm9, ymm9, ymmword ptr [rbp-0x1E0]
       vmovups  ymm14, ymmword ptr [rbp-0x2430]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1C0]
       vaddps   ymm9, ymm14, ymm9
       vmovups  ymm14, ymmword ptr [rbp-0x2450]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1A0]
       vaddps   ymm9, ymm14, ymm9
       vmovups  ymmword ptr [rbp-0x24D0], ymm9
       vmovups  ymm14, ymmword ptr [rbp-0x2410]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x180]
       vmovups  ymm9, ymmword ptr [rbp-0x2430]
       vmulps   ymm9, ymm9, ymmword ptr [rbp-0x160]
       vaddps   ymm9, ymm9, ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2450]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x140]
       vaddps   ymm9, ymm14, ymm9
       vmovups  ymmword ptr [rbp-0x24F0], ymm9
       vmovups  ymm14, ymmword ptr [rbp-0x2410]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x120]
       vmovups  ymm9, ymmword ptr [rbp-0x2430]
       vmulps   ymm9, ymm9, ymmword ptr [rbp-0x100]
       vaddps   ymm9, ymm9, ymm14
 
G_M000_IG07:                ;; offset=0x075E
       vmovups  ymm14, ymmword ptr [rbp-0x2450]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0xE0]
       vaddps   ymm9, ymm14, ymm9
       vmovups  ymmword ptr [rbp-0x2510], ymm9
       vxorps   ymm14, ymm14, ymm14
       vmovups  ymm9, ymmword ptr [rbp-0x24F0]
       vcmpgtps ymm9, ymm9, ymm14
       vmovups  ymm14, ymmword ptr [rsi+0x20]
       vmovups  ymmword ptr [rbp-0x3010], ymm14
       vxorps   ymm14, ymm2, ymm14
       vblendvps ymm9, ymm14, ymmword ptr [rbp-0x3010], ymm9
       vmovups  ymmword ptr [rbp-0x2550], ymm9
       vmovups  ymm14, ymmword ptr [rbp-0x24D0]
       vmulps   ymm14, ymm14, ymm14
       vmovups  ymm9, ymmword ptr [rbp-0x2510]
       vmulps   ymm9, ymm9, ymm9
       vaddps   ymm9, ymm9, ymm14
       vsqrtps  ymm9, ymm9
       vmovups  ymm14, ymmword ptr [rsi]
       vdivps   ymm14, ymm14, ymm9
       vmovups  ymmword ptr [rbp-0xE30], ymm14
       vcmpgtps ymm9, ymm9, ymmword ptr [reloc @RWD128]
       vmovups  ymm14, ymmword ptr [rbp-0x24D0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0xE30]
       vandps   ymm14, ymm14, ymm9
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vandnps  ymm14, ymm9, ymm14
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x2530], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2510]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0xE30]
       vandps   ymm14, ymm14, ymm9
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vandnps  ymm9, ymm9, ymm14
       vorps    ymm9, ymm9, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x2570], ymm9
       vmovups  ymm14, ymmword ptr [rbp-0x2530]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1E0]
       vmovups  ymm9, ymmword ptr [rbp-0x2550]
       vmulps   ymm9, ymm9, ymmword ptr [rbp-0x180]
       vaddps   ymm9, ymm9, ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2570]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x120]
       vaddps   ymm9, ymm14, ymm9
       vmovups  ymmword ptr [rbp-0x2470], ymm9
       vmovups  ymm14, ymmword ptr [rbp-0x2530]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1C0]
       vmovups  ymm9, ymmword ptr [rbp-0x2550]
       vmulps   ymm9, ymm9, ymmword ptr [rbp-0x160]
       vaddps   ymm9, ymm9, ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2570]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x100]
       vaddps   ymm9, ymm14, ymm9
       vmovups  ymmword ptr [rbp-0x2490], ymm9
       vmovups  ymm14, ymmword ptr [rbp-0x2530]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1A0]
       vmovups  ymm9, ymmword ptr [rbp-0x2550]
 
G_M000_IG08:                ;; offset=0x08FA
       vmulps   ymm9, ymm9, ymmword ptr [rbp-0x140]
       vaddps   ymm9, ymm9, ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2570]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0xE0]
       vaddps   ymm9, ymm14, ymm9
       vmovups  ymm14, ymmword ptr [rbp-0x2470]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x240]
       vmovups  ymmword ptr [rbp-0x2470], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2490]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x220]
       vmovups  ymmword ptr [rbp-0x2490], ymm14
       vaddps   ymm9, ymm9, ymmword ptr [rbp-0x200]
       vmovups  ymmword ptr [rbp-0x24B0], ymm9
       vsubps   ymm14, ymm11, ymmword ptr [rbp-0x2470]
       vsubps   ymm9, ymm4, ymmword ptr [rbp-0x2490]
       vsubps   ymm5, ymm5, ymmword ptr [rbp-0x24B0]
       vmulps   ymm4, ymm14, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x3310], ymm4
       vmulps   ymm4, ymm9, ymmword ptr [rbp-0x280]
       vaddps   ymm4, ymm4, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm4
       vmulps   ymm4, ymm5, ymmword ptr [rbp-0x260]
       vaddps   ymm4, ymm4, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0xAF0], ymm4
       vmovups  ymmword ptr [rbp-0xE10], ymm14
       vmovups  ymmword ptr [rbp-0xDF0], ymm9
       vmovups  ymmword ptr [rbp-0xDD0], ymm5
       vmovups  ymmword ptr [rbp-0xD10], ymm14
       vmovups  ymmword ptr [rbp-0xCF0], ymm9
       vmovups  ymmword ptr [rbp-0xCD0], ymm5
       vmovups  ymmword ptr [rbp-0xC10], ymm14
       vmovups  ymmword ptr [rbp-0xBF0], ymm9
       vmovups  ymmword ptr [rbp-0xBD0], ymm5
       vmovups  ymmword ptr [rbp-0xDB0], ymm11
       vmovups  ymm4, ymmword ptr [rbp-0x23D0]
       vmovups  ymmword ptr [rbp-0xD90], ymm4
       vmovups  ymm5, ymmword ptr [rbp-0x23F0]
       vmovups  ymmword ptr [rbp-0xD70], ymm5
       vmovups  ymmword ptr [rbp-0xCB0], ymm11
       vmovups  ymmword ptr [rbp-0xC90], ymm4
       vmovups  ymmword ptr [rbp-0xC70], ymm5
       vmovups  ymmword ptr [rbp-0xBB0], ymm11
       vmovups  ymmword ptr [rbp-0xB90], ymm4
       vmovups  ymmword ptr [rbp-0xB70], ymm5
       vpcmpeqd ymm4, ymm4, ymm4
       vmovups  ymmword ptr [rbp-0xD30], ymm4
       vxorps   ymm4, ymm4, ymm4
       vmovups  ymmword ptr [rbp-0xC30], ymm4
       vxorps   ymm4, ymm4, ymm4
       vmovups  ymmword ptr [rbp-0xB30], ymm4
       lea      r8, [rbp-0x2A8]
       mov      qword ptr [rsp+0x20], r8
       lea      r8, [rbp-0x2A8]
       mov      qword ptr [rsp+0x28], r8
       lea      r8, [rbp-0xE10]
       mov      qword ptr [rsp+0x30], r8
       lea      r8, [rbp-0x2A0]
       mov      qword ptr [rsp+0x38], r8
       lea      r8, [rbp-0xAF0]
 
G_M000_IG09:                ;; offset=0x0AAF
       mov      qword ptr [rsp+0x40], r8
       lea      r8, [rbp-0x2D0]
       mov      qword ptr [rsp+0x48], r8
       lea      r8, [rbp-0x570]
       mov      qword ptr [rsp+0x50], r8
       lea      r8, [rbp-0x2F0]
       mov      qword ptr [rsp+0x58], r8
       lea      r8, [rbp-0x310]
       mov      qword ptr [rsp+0x60], r8
       lea      r8, [rbp-0x2A0]
       mov      qword ptr [rsp+0x68], r8
       lea      r8, [rbp-0x370]
       mov      qword ptr [rsp+0x70], r8
       mov      dword ptr [rsp+0x78], 25
       lea      r8, [rbp-0x240]
       lea      r9, [rbp-0x1E0]
       mov      rcx, rdi
       mov      rdx, rsi
       vextractf128 xmm9, ymm8
       vextractf128 xmm11, ymm15
       vextractf128 xmm14, ymm7
       vextractf128 xmmword ptr [rbp-0x2E60], ymm12
       vextractf128 xmmword ptr [rbp-0x2E80], ymm13
       vextractf128 xmmword ptr [rbp-0x2EA0], ymm6
       vextractf128 xmmword ptr [rbp-0x2EC0], ymm10
       call     [BepuPhysics.CollisionDetection.DepthRefiner`6[BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder,BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder]:FindMinimumDepth(byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,int)]
       vmovups  ymm0, ymmword ptr [rbp-0x310]
       vcmpltps ymm0, ymm0, ymmword ptr [rbp-0x2F0]
       vpor     ymm0, ymm0, ymmword ptr [rbp-0x2D0]
       vmovups  ymmword ptr [rbp-0x2D0], ymm0
       vxorps   ymm0, ymm0, ymm0
       vpcmpgtd ymm0, ymm0, ymmword ptr [rbp-0x2D0]
       vpcmpeqd ymm1, ymm1, ymm1
       vptest   ymm0, ymm1
       vinsertf128 ymm8, ymm8, xmm9
       vinsertf128 ymm15, ymm15, xmm11
       vinsertf128 ymm7, ymm7, xmm14
       vinsertf128 ymm12, ymm12, xmmword ptr [rbp-0x2E60]
       vinsertf128 ymm13, ymm13, xmmword ptr [rbp-0x2E80]
       vinsertf128 ymm6, ymm6, xmmword ptr [rbp-0x2EA0]
       vinsertf128 ymm10, ymm10, xmmword ptr [rbp-0x2EC0]
       jb       G_M000_IG49
 
G_M000_IG10:                ;; offset=0x0BCE
       vmovups  ymm0, ymmword ptr [rbp-0x180]
       vmulps   ymm0, ymm0, ymmword ptr [rbp-0x2A0]
       vmovups  ymm1, ymmword ptr [rbp-0x160]
       vmulps   ymm1, ymm1, ymmword ptr [rbp-0x280]
       vaddps   ymm0, ymm1, ymm0
       vmovups  ymm1, ymmword ptr [rbp-0x140]
       vmulps   ymm1, ymm1, ymmword ptr [rbp-0x260]
       vaddps   ymm0, ymm1, ymm0
       vmovups  ymmword ptr [rbp-0x390], ymm0
       vdivps   ymm1, ymm8, ymm0
       vmovups  ymmword ptr [rbp-0x3B0], ymm1
       vdivps   ymm2, ymm8, ymmword ptr [rbp-0x280]
       vmovups  ymmword ptr [rbp-0x3D0], ymm2
       vxorps   ymm3, ymm3, ymm3
       vcmpgtps ymm3, ymm0, ymm3
       vmovups  ymm4, ymmword ptr [rsi+0x20]
       vmovups  ymm9, ymmword ptr [rbp-0x3050]
       vxorps   ymm5, ymm9, ymm4
       vblendvps ymm3, ymm4, ymm5, ymm3
       vmovups  ymmword ptr [rbp-0x570], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x180]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x570]
       vmovups  ymm4, ymmword ptr [rbp-0x160]
       vmulps   ymm4, ymm4, ymmword ptr [rbp-0x570]
       vmovups  ymm5, ymmword ptr [rbp-0x140]
       vmulps   ymm5, ymm5, ymmword ptr [rbp-0x570]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x240]
       vmovups  ymmword ptr [rbp-0x14D0], ymm3
       vaddps   ymm11, ymm4, ymmword ptr [rbp-0x220]
       vmovups  ymmword ptr [rbp-0x2FF0], ymm11
       vmovups  ymmword ptr [rbp-0x14F0], ymm11
       vaddps   ymm5, ymm5, ymmword ptr [rbp-0x200]
       vmovups  ymm14, ymmword ptr [rbp-0x280]
       vxorps   ymm11, ymm11, ymm11
       vcmpltps ymm11, ymm14, ymm11
       vmovups  ymm14, ymmword ptr [rdi+0x20]
       vmovups  ymmword ptr [rbp-0x3030], ymm14
       vxorps   ymm14, ymm9, ymm14
       vmovups  ymm2, ymmword ptr [rbp-0x3030]
       vblendvps ymm2, ymm2, ymm14, ymm11
       vbroadcastss ymm11, dword ptr [reloc @RWD160]
       vmovups  ymmword ptr [rbp-0x3F0], ymm11
       vbroadcastss ymm14, dword ptr [reloc @RWD164]
       vmovups  ymmword ptr [rbp-0x3070], ymm14
       vandps   ymm11, ymm14, ymm0
       vmovups  ymmword ptr [rbp-0x2F90], ymm11
       vcmpgtps ymm11, ymm11, ymmword ptr [rbp-0x3F0]
       vandps   ymm0, ymm14, ymmword ptr [rbp-0x280]
       vcmpgtps ymm0, ymm0, ymmword ptr [rbp-0x3F0]
       vmovups  ymmword ptr [rbp-0x410], ymm0
       vxorps   ymm0, ymm0, ymm0
       vmovups  ymmword ptr [rbx+0x2E0], ymm0
       vmovups  ymmword ptr [rbx+0x300], ymm0
       vxorps   ymm0, ymm0, ymm0
       vmovups  ymmword ptr [rbx+0x320], ymm0
       vmovups  ymmword ptr [rbx+0x340], ymm0
       vpand    ymm0, ymm11, ymmword ptr [rbp-0x410]
       vmovups  ymmword ptr [rbp-0x3310], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x2D0]
 
G_M000_IG11:                ;; offset=0x0D75
       vpandn   ymm0, ymm0, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x430], ymm0
       vxorps   ymm0, ymm9, ymmword ptr [rbp-0x310]
       vmovups  ymmword ptr [rbp-0x570], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x2A0]
       vmulps   ymm0, ymm0, ymmword ptr [rbp-0x570]
       vmovups  ymmword ptr [rbp-0x16B0], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x280]
       vmulps   ymm0, ymm0, ymmword ptr [rbp-0x570]
       vmovups  ymmword ptr [rbp-0x16D0], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x260]
       vmulps   ymm0, ymm0, ymmword ptr [rbp-0x570]
       vmovups  ymmword ptr [rbp-0x16F0], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x16B0]
       vaddps   ymm0, ymm0, ymmword ptr [rbp-0x370]
       vmovups  ymmword ptr [rbp-0x1710], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x16D0]
       vaddps   ymm0, ymm0, ymmword ptr [rbp-0x350]
       vmovups  ymmword ptr [rbp-0x1730], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x16F0]
       vaddps   ymm0, ymm0, ymmword ptr [rbp-0x330]
       vmovups  ymmword ptr [rbp-0x1750], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x1710]
       vsubps   ymm0, ymm0, ymmword ptr [rbp-0x240]
       vmovups  ymmword ptr [rbp-0x1770], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x1730]
       vsubps   ymm0, ymm0, ymmword ptr [rbp-0x220]
       vmovups  ymmword ptr [rbp-0x1790], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x1750]
       vsubps   ymm0, ymm0, ymmword ptr [rbp-0x200]
       vmovups  ymmword ptr [rbp-0x17B0], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x180]
       vmovups  ymmword ptr [rbp-0x3090], ymm0
       vmulps   ymm1, ymm0, ymmword ptr [rbp-0x1770]
       vmovups  ymmword ptr [rbp-0x3310], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x160]
       vmovups  ymmword ptr [rbp-0x30B0], ymm1
       vmulps   ymm1, ymm1, ymmword ptr [rbp-0x1790]
       vaddps   ymm1, ymm1, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x140]
       vmovups  ymmword ptr [rbp-0x30D0], ymm1
       vmulps   ymm4, ymm1, ymmword ptr [rbp-0x17B0]
       vaddps   ymm4, ymm4, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x450], ymm4
       vmulps   ymm4, ymm4, ymm0
       vmovups  ymmword ptr [rbp-0x17D0], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x450]
       vmulps   ymm4, ymm4, ymmword ptr [rbp-0x30B0]
       vmovups  ymmword ptr [rbp-0x17F0], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x450]
       vmulps   ymm4, ymm4, ymm1
       vmovups  ymmword ptr [rbp-0x1810], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x1770]
       vsubps   ymm4, ymm4, ymmword ptr [rbp-0x17D0]
       vmovups  ymmword ptr [rbp-0x1770], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x1790]
       vsubps   ymm4, ymm4, ymmword ptr [rbp-0x17F0]
       vmovups  ymmword ptr [rbp-0x1790], ymm4
 
G_M000_IG12:                ;; offset=0x0F45
       vmovups  ymm4, ymmword ptr [rbp-0x17B0]
       vsubps   ymm4, ymm4, ymmword ptr [rbp-0x1810]
       vmovups  ymmword ptr [rbp-0x17B0], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x370]
       vmovups  ymmword ptr [rbp-0x1830], ymm4
       vmovups  ymm3, ymmword ptr [rbp-0x330]
       vmovups  ymmword ptr [rbp-0x1850], ymm3
       vxorps   ymm1, ymm1, ymm1
       vmovups  ymmword ptr [rbp-0x3310], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x390]
       vcmpgtps ymm1, ymm1, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x470], ymm1
       vxorps   ymm1, ymm9, ymm0
       vandps   ymm1, ymm1, ymmword ptr [rbp-0x470]
       vmovups  ymmword ptr [rbp-0x3310], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x470]
       vandnps  ymm1, ymm1, ymm0
       vorps    ymm1, ymm1, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1870], ymm1
       vxorps   ymm1, ymm9, ymmword ptr [rbp-0x30B0]
       vandps   ymm1, ymm1, ymmword ptr [rbp-0x470]
       vmovups  ymmword ptr [rbp-0x3310], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x470]
       vandnps  ymm1, ymm1, ymmword ptr [rbp-0x30B0]
       vorps    ymm1, ymm1, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1890], ymm1
       vxorps   ymm1, ymm9, ymmword ptr [rbp-0x30D0]
       vandps   ymm1, ymm1, ymmword ptr [rbp-0x470]
       vmovups  ymmword ptr [rbp-0x3310], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x470]
       vandnps  ymm1, ymm1, ymmword ptr [rbp-0x30D0]
       vorps    ymm1, ymm1, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x18B0], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x1870]
       vmovups  ymmword ptr [rbp-0x18D0], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x1890]
       vmovups  ymmword ptr [rbp-0x18F0], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x18B0]
       vmovups  ymmword ptr [rbp-0x1910], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x14D0]
       vmovups  ymm0, ymmword ptr [rbp-0x14F0]
       vmovups  ymmword ptr [rbp-0x1510], ymm5
       vxorps   ymm4, ymm4, ymm4
       vpcmpgtd ymm4, ymm4, ymmword ptr [rbp-0x430]
       vptest   ymm4, ymm4
       jne      SHORT G_M000_IG14
 
G_M000_IG13:                ;; offset=0x109D
       jmp      G_M000_IG30
 
G_M000_IG14:                ;; offset=0x10A2
       vbroadcastss ymm4, dword ptr [reloc @RWD168]
       vmovups  ymmword ptr [rbp-0x590], ymm4
       vbroadcastss ymm4, dword ptr [reloc @RWD172]
       vmovups  ymmword ptr [rbp-0x5B0], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x2F90]
       vmovups  ymmword ptr [rbp-0x5D0], ymm4
       vandps   ymm4, ymm14, ymmword ptr [rbp-0x280]
       vmovups  ymmword ptr [rbp-0x5F0], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x5D0]
       vcmpltps ymm4, ymm4, ymmword ptr [rbp-0x590]
       vmovups  ymmword ptr [rbp-0x610], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x5F0]
       vcmpltps ymm4, ymm4, ymmword ptr [rbp-0x590]
       vmovups  ymmword ptr [rbp-0x630], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x1830]
       vmovups  ymmword ptr [rbp-0x1AD0], ymm4
       vmovups  ymmword ptr [rbp-0x1AF0], ymm3
       vmovups  ymm4, ymmword ptr [rbp-0x610]
       vpand    ymm4, ymm4, ymmword ptr [rbp-0x630]
       vmovups  ymmword ptr [rbp-0x650], ymm4
       vpcmpeqd ymm4, ymm4, ymm4
       vpxor    ymm4, ymm4, ymmword ptr [rbp-0x650]
       vmovups  ymmword ptr [rbp-0x2FD0], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x2D0]
       vpandn   ymm4, ymm4, ymmword ptr [rbp-0x2FD0]
       vmovups  ymmword ptr [rbp-0x3310], ymm4
       vxorps   ymm4, ymm4, ymm4
       vpcmpgtd ymm4, ymm4, ymmword ptr [rbp-0x3310]
       vptest   ymm4, ymm4
       je       G_M000_IG29
       vsubps   ymm4, ymm0, ymm2
       vmulps   ymm4, ymm4, ymmword ptr [rbp-0x3D0]
       vmovups  ymmword ptr [rbp-0xE50], ymm4
       vmulps   ymm4, ymm4, ymmword ptr [rbp-0x2A0]
       vsubps   ymm4, ymm1, ymm4
       vmovups  ymmword ptr [rbp-0x1B10], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0xE50]
       vmulps   ymm4, ymm4, ymmword ptr [rbp-0x260]
       vsubps   ymm4, ymm5, ymm4
       vmovups  ymmword ptr [rbp-0x1B30], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x1B10]
       vmulps   ymm4, ymm4, ymm4
       vmovups  ymmword ptr [rbp-0x3310], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x1B30]
       vmulps   ymm4, ymm4, ymm4
       vaddps   ymm4, ymm4, ymmword ptr [rbp-0x3310]
       vsqrtps  ymm4, ymm4
       vmovups  ymmword ptr [rbp-0x670], ymm4
       vdivps   ymm4, ymm8, ymm4
       vmovups  ymmword ptr [rbp-0x690], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x1B10]
       vmulps   ymm4, ymm4, ymmword ptr [rbp-0x690]
       vmovups  ymmword ptr [rbp-0x1B50], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x1B30]
       vmulps   ymm4, ymm4, ymmword ptr [rbp-0x690]
       vmovups  ymm14, ymmword ptr [rbp-0x670]
       vcmpltps ymm14, ymm14, ymmword ptr [reloc @RWD192]
       vmovups  ymmword ptr [rbp-0x6B0], ymm14
 
G_M000_IG15:                ;; offset=0x124E
       vandps   ymm14, ymm8, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x6B0]
       vandnps  ymm14, ymm14, ymmword ptr [rbp-0x1B50]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1B50], ymm14
       vxorps   ymm14, ymm14, ymm14
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x6B0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x6B0]
       vandnps  ymm4, ymm14, ymm4
       vorps    ymm4, ymm4, ymmword ptr [rbp-0x3310]
       vmovups  ymm14, ymmword ptr [rdi]
       vmovups  ymmword ptr [rbp-0x30F0], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1B50]
       vmovups  ymmword ptr [rbp-0x1B70], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x30F0]
       vmulps   ymm14, ymm14, ymm4
       vmovups  ymmword ptr [rbp-0x1B90], ymm14
       vxorps   ymm14, ymm9, ymmword ptr [rbp-0x1B70]
       vmovups  ymmword ptr [rbp-0x1BB0], ymm14
       vxorps   ymm14, ymm9, ymmword ptr [rbp-0x1B90]
       vmovups  ymmword ptr [rbp-0x1BD0], ymm14
       vsubps   ymm14, ymm1, ymmword ptr [rbp-0x1B70]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x3090]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vsubps   ymm14, ymm0, ymm2
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x30B0]
       vmovups  ymmword ptr [rbp-0x2EF0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3310]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x2EF0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vsubps   ymm14, ymm5, ymmword ptr [rbp-0x1B90]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x30D0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0xE70], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3B0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0xE70]
       vmovups  ymmword ptr [rbp-0xE90], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x2590], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xE90]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x280]
       vmovups  ymmword ptr [rbp-0x25B0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xE90]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x260]
       vmovups  ymmword ptr [rbp-0x25D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1B70]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x2590]
       vmovups  ymmword ptr [rbp-0x25F0], ymm14
       vaddps   ymm14, ymm2, ymmword ptr [rbp-0x25B0]
       vmovups  ymmword ptr [rbp-0x2610], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1B90]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x25D0]
       vmovups  ymmword ptr [rbp-0x2630], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x25F0]
       vsubps   ymm14, ymm14, ymm1
       vmovups  ymmword ptr [rbp-0x2650], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2610]
 
G_M000_IG16:                ;; offset=0x140C
       vsubps   ymm14, ymm14, ymm0
       vmovups  ymmword ptr [rbp-0x2670], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2630]
       vsubps   ymm14, ymm14, ymm5
       vmovups  ymmword ptr [rbp-0x2690], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1E0]
       vmovups  ymmword ptr [rbp-0x3110], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2650]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1C0]
       vmovups  ymmword ptr [rbp-0x3130], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2670]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1A0]
       vmovups  ymmword ptr [rbp-0x3150], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2690]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1BF0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x120]
       vmovups  ymmword ptr [rbp-0x3170], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2650]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x100]
       vmovups  ymmword ptr [rbp-0x3190], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2670]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xE0]
       vmovups  ymmword ptr [rbp-0x31B0], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2690]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1C10], ymm14
       vsubps   ymm14, ymm1, ymmword ptr [rbp-0x1BB0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x3090]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x2EF0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vsubps   ymm14, ymm5, ymmword ptr [rbp-0x1BD0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x30D0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0xEB0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3B0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0xEB0]
       vmovups  ymmword ptr [rbp-0xED0], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x26B0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xED0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x280]
       vmovups  ymmword ptr [rbp-0x26D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xED0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x260]
       vmovups  ymmword ptr [rbp-0x26F0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1BB0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x26B0]
       vmovups  ymmword ptr [rbp-0x2710], ymm14
       vaddps   ymm14, ymm2, ymmword ptr [rbp-0x26D0]
       vmovups  ymmword ptr [rbp-0x2730], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1BD0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x26F0]
 
G_M000_IG17:                ;; offset=0x15DC
       vmovups  ymmword ptr [rbp-0x2750], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2710]
       vsubps   ymm14, ymm14, ymm1
       vmovups  ymmword ptr [rbp-0x2770], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2730]
       vsubps   ymm14, ymm14, ymm0
       vmovups  ymmword ptr [rbp-0x2790], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2750]
       vsubps   ymm14, ymm14, ymm5
       vmovups  ymmword ptr [rbp-0x27B0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3110]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2770]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3130]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2790]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3150]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x27B0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1C30], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3170]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2770]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3190]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2790]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x31B0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x27B0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1C50], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1C30]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x1BF0]
       vmovups  ymmword ptr [rbp-0x1C70], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1C50]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x1C10]
       vmovups  ymmword ptr [rbp-0x1C90], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1BB0]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x1B70]
       vmovups  ymmword ptr [rbp-0x1CB0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1BD0]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x1B90]
       vmovups  ymmword ptr [rbp-0x1CD0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1C70]
       vmulps   ymm14, ymm14, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1C90]
       vmulps   ymm14, ymm14, ymm14
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0xEF0], ymm14
       vdivps   ymm14, ymm8, ymm14
       vmovups  ymmword ptr [rbp-0xF10], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1BF0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1C70]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1C10]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1C90]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
 
G_M000_IG18:                ;; offset=0x179F
       vmovups  ymmword ptr [rbp-0xF30], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1BF0]
       vmulps   ymm14, ymm14, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1C10]
       vmulps   ymm14, ymm14, ymm14
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rsi]
       vmulps   ymm14, ymm14, ymmword ptr [rsi]
       vmovups  ymmword ptr [rbp-0x2F10], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3310]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x2F10]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0xEF0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xF30]
       vmulps   ymm14, ymm14, ymm14
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0xF50], ymm14
       vxorps   ymm14, ymm14, ymm14
       vcmpeqps ymm14, ymm14, ymmword ptr [rbp-0xF50]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymmword ptr [rbp-0xF50]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xF50]
       vcmpltps ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vandnps  ymm14, ymm14, ymmword ptr [rbp-0xF50]
       vsqrtps  ymm14, ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0xF10]
       vmovups  ymmword ptr [rbp-0xF70], ymm14
       vxorps   ymm14, ymm9, ymmword ptr [rbp-0xF30]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0xF10]
       vmovups  ymmword ptr [rbp-0xF90], ymm14
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0xF70]
       vmovups  ymmword ptr [rbp-0x6D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xF90]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0xF70]
       vmovups  ymmword ptr [rbp-0x6F0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3070]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0xEF0]
       vcmpltps ymm14, ymm14, ymmword ptr [reloc @RWD224]
       vmovups  ymmword ptr [rbp-0xFB0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xF90]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0xFB0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xFB0]
       vandnps  ymm14, ymm14, ymmword ptr [rbp-0x6D0]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x6D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xF90]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0xFB0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xFB0]
       vandnps  ymm14, ymm14, ymmword ptr [rbp-0x6F0]
 
G_M000_IG19:                ;; offset=0x195D
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x6F0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x6D0]
       vcmpneqps ymm14, ymm14, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x6D0]
       vcmpgtps ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x6D0]
       vmovups  ymmword ptr [rbp-0x710], ymm14
       vminps   ymm14, ymm8, ymmword ptr [rbp-0x6F0]
       vmovups  ymmword ptr [rbp-0x730], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x710]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1CB0]
       vmovups  ymmword ptr [rbp-0x1AD0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x710]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1CD0]
       vmovups  ymmword ptr [rbp-0x1AF0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1B70]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x1AD0]
       vmovups  ymmword ptr [rbp-0x1AD0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1B90]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x1AF0]
       vmovups  ymmword ptr [rbp-0x1AF0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x730]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1CB0]
       vmovups  ymmword ptr [rbp-0x1CF0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x730]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1CD0]
       vmovups  ymmword ptr [rbp-0x1D10], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1B70]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x1CF0]
       vmovups  ymmword ptr [rbp-0x1CF0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1B90]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x1D10]
       vmovups  ymmword ptr [rbp-0x1D10], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x30F0]
       vmulps   ymm14, ymm14, ymm14
       vmovups  ymmword ptr [rbp-0x2F30], ymm14
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x2F10]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x690]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x670]
       vmulps   ymm14, ymm14, ymmword ptr [reloc @RWD256]
       vmovups  ymmword ptr [rbp-0x750], ymm14
       vxorps   ymm14, ymm14, ymm14
       vcmpeqps ymm14, ymm14, ymmword ptr [rbp-0x750]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymmword ptr [rbp-0x750]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x750]
       vcmpltps ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vandnps  ymm14, ymm14, ymmword ptr [rbp-0x750]
 
G_M000_IG20:                ;; offset=0x1B27
       vmovups  ymmword ptr [rbp-0x9D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x670]
       vcmpeqps ymm14, ymm14, ymmword ptr [rbp-0x9D0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymmword ptr [rbp-0x670]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x670]
       vcmpneqps ymm14, ymm14, ymm14
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x670]
       vcmpltps ymm14, ymm14, ymmword ptr [rbp-0x9D0]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymm3, ymmword ptr [rbp-0x9D0]
       vblendvps ymm3, ymm3, ymmword ptr [rbp-0x670], ymm14
       vmovups  ymmword ptr [rbp-0x770], ymm3
       vmovups  ymm14, ymmword ptr [rbp-0x1B50]
       vmulps   ymm3, ymm14, ymm3
       vmovups  ymmword ptr [rbp-0x1D30], ymm3
       vmulps   ymm14, ymm4, ymmword ptr [rbp-0x770]
       vmovups  ymmword ptr [rbp-0x1D50], ymm14
       vxorps   ymm14, ymm9, ymmword ptr [rbp-0x1B50]
       vmovups  ymmword ptr [rbp-0x1D70], ymm14
       vaddps   ymm14, ymm4, ymm3
       vmovups  ymmword ptr [rbp-0x1D90], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1D50]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x1D70]
       vmovups  ymmword ptr [rbp-0x1DB0], ymm14
       vsubps   ymm14, ymm1, ymm3
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x3090]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x2EF0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vsubps   ymm14, ymm5, ymmword ptr [rbp-0x1D50]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x30D0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0xFD0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3B0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0xFD0]
       vmovups  ymmword ptr [rbp-0xFF0], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x27D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xFF0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x280]
       vmovups  ymmword ptr [rbp-0x27F0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xFF0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x260]
       vmovups  ymmword ptr [rbp-0x2810], ymm14
       vaddps   ymm14, ymm3, ymmword ptr [rbp-0x27D0]
       vmovups  ymmword ptr [rbp-0x2830], ymm14
       vaddps   ymm14, ymm2, ymmword ptr [rbp-0x27F0]
       vmovups  ymmword ptr [rbp-0x2850], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1D50]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x2810]
       vmovups  ymmword ptr [rbp-0x2870], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2830]
       vsubps   ymm14, ymm14, ymm1
 
G_M000_IG21:                ;; offset=0x1CE6
       vmovups  ymmword ptr [rbp-0x2890], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2850]
       vsubps   ymm14, ymm14, ymm0
       vmovups  ymmword ptr [rbp-0x28B0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2870]
       vsubps   ymm14, ymm14, ymm5
       vmovups  ymmword ptr [rbp-0x28D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3110]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2890]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3130]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x28B0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3150]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x28D0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1DD0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3170]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2890]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3190]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x28B0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x31B0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x28D0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1DF0], ymm14
       vsubps   ymm14, ymm1, ymmword ptr [rbp-0x1D90]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x3090]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x2EF0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vsubps   ymm14, ymm5, ymmword ptr [rbp-0x1DB0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x30D0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1010], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3B0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1010]
       vmovups  ymmword ptr [rbp-0x1030], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x28F0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1030]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x280]
       vmovups  ymmword ptr [rbp-0x2910], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1030]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x260]
       vmovups  ymmword ptr [rbp-0x2930], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1D90]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x28F0]
       vmovups  ymmword ptr [rbp-0x2950], ymm14
       vaddps   ymm14, ymm2, ymmword ptr [rbp-0x2910]
       vmovups  ymmword ptr [rbp-0x2970], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1DB0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x2930]
       vmovups  ymmword ptr [rbp-0x2990], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2950]
       vsubps   ymm14, ymm14, ymm1
       vmovups  ymmword ptr [rbp-0x29B0], ymm14
 
G_M000_IG22:                ;; offset=0x1EB2
       vmovups  ymm14, ymmword ptr [rbp-0x2970]
       vsubps   ymm14, ymm14, ymm0
       vmovups  ymmword ptr [rbp-0x29D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2990]
       vsubps   ymm14, ymm14, ymm5
       vmovups  ymmword ptr [rbp-0x29F0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3110]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x29B0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3130]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x29D0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3150]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x29F0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1E10], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3170]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x29B0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3190]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x29D0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x31B0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x29F0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1E30], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1E10]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x1DD0]
       vmovups  ymmword ptr [rbp-0x1E50], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1E30]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x1DF0]
       vmovups  ymmword ptr [rbp-0x1E70], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1E50]
       vmulps   ymm14, ymm14, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1E70]
       vmulps   ymm14, ymm14, ymm14
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1050], ymm14
       vdivps   ymm14, ymm8, ymm14
       vmovups  ymmword ptr [rbp-0x1070], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1DD0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1E50]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1DF0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1E70]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1090], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1DD0]
       vmulps   ymm14, ymm14, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1DF0]
       vmulps   ymm14, ymm14, ymm14
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x2F10]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1050]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
 
G_M000_IG23:                ;; offset=0x2073
       vmovups  ymm14, ymmword ptr [rbp-0x1090]
       vmulps   ymm14, ymm14, ymm14
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vxorps   ymm3, ymm3, ymm3
       vcmpeqps ymm3, ymm3, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm3, ymm3, ymm14
       vandps   ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vxorps   ymm3, ymm3, ymm3
       vcmpltps ymm3, ymm14, ymm3
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vandnps  ymm3, ymm3, ymm14
       vsqrtps  ymm3, ymm3
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x1070]
       vmovups  ymmword ptr [rbp-0x10B0], ymm3
       vxorps   ymm14, ymm9, ymmword ptr [rbp-0x1090]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1070]
       vmovups  ymmword ptr [rbp-0x10D0], ymm14
       vsubps   ymm3, ymm14, ymm3
       vmovups  ymmword ptr [rbp-0x790], ymm3
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x10B0]
       vmovups  ymm3, ymmword ptr [rbp-0x3070]
       vandps   ymm3, ymm3, ymmword ptr [rbp-0x1050]
       vcmpltps ymm3, ymm3, ymmword ptr [reloc @RWD224]
       vmovups  ymmword ptr [rbp-0x10F0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x10D0]
       vandps   ymm3, ymm3, ymmword ptr [rbp-0x10F0]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x10F0]
       vandnps  ymm3, ymm3, ymmword ptr [rbp-0x790]
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x790], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x10D0]
       vandps   ymm3, ymm3, ymmword ptr [rbp-0x10F0]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x10F0]
       vandnps  ymm3, ymm3, ymm14
       vorps    ymm14, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x7B0], ymm14
       vmulps   ymm3, ymm4, ymm4
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1D70]
       vmulps   ymm3, ymm3, ymm3
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1110], ymm3
       vdivps   ymm3, ymm8, ymm3
       vmovups  ymmword ptr [rbp-0x1130], ymm3
       vmulps   ymm3, ymm4, ymmword ptr [rbp-0x1D30]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1D50]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x1D70]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1150], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1D30]
       vmulps   ymm3, ymm3, ymm3
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1D50]
 
G_M000_IG24:                ;; offset=0x2217
       vmulps   ymm3, ymm3, ymm3
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vsubps   ymm3, ymm3, ymmword ptr [rbp-0x2F30]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x1110]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1150]
       vmulps   ymm3, ymm3, ymm3
       vsubps   ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vxorps   ymm14, ymm14, ymm14
       vcmpeqps ymm14, ymm14, ymm3
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymm3
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vcmpltps ymm14, ymm3, ymm14
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vandnps  ymm3, ymm14, ymm3
       vsqrtps  ymm3, ymm3
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x1130]
       vmovups  ymmword ptr [rbp-0x1170], ymm3
       vxorps   ymm14, ymm9, ymmword ptr [rbp-0x1150]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1130]
       vmovups  ymmword ptr [rbp-0x1190], ymm14
       vsubps   ymm3, ymm14, ymm3
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x1170]
       vmovups  ymmword ptr [rbp-0x7D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3070]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x1110]
       vcmpltps ymm14, ymm14, ymmword ptr [reloc @RWD224]
       vmovups  ymmword ptr [rbp-0x11B0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1190]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x11B0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x11B0]
       vandnps  ymm3, ymm14, ymm3
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymm14, ymmword ptr [rbp-0x1190]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x11B0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x11B0]
       vandnps  ymm14, ymm14, ymmword ptr [rbp-0x7D0]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x7D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x790]
       vcmpeqps ymm14, ymm14, ymm3
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymm3
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x790]
       vcmpneqps ymm14, ymm14, ymm14
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vcmpltps ymm14, ymm3, ymmword ptr [rbp-0x790]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vblendvps ymm3, ymm3, ymmword ptr [rbp-0x790], ymm14
 
G_M000_IG25:                ;; offset=0x23BD
       vmovups  ymmword ptr [rbp-0x7F0], ymm3
       vmovups  ymm14, ymmword ptr [rbp-0x7B0]
       vcmpeqps ymm3, ymm14, ymmword ptr [rbp-0x7D0]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm3, ymm3, ymm14
       vandps   ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vcmpneqps ymm3, ymm14, ymm14
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vmovups  ymmword ptr [rbp-0x7B0], ymm14
       vcmpltps ymm3, ymm14, ymmword ptr [rbp-0x7D0]
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymm14, ymmword ptr [rbp-0x7D0]
       vblendvps ymm3, ymm14, ymmword ptr [rbp-0x7B0], ymm3
       vmovups  ymmword ptr [rbp-0x810], ymm3
       vmulps   ymm14, ymm4, ymmword ptr [rbp-0x7F0]
       vmovups  ymm3, ymmword ptr [rbp-0x1D70]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x7F0]
       vmovups  ymmword ptr [rbp-0x1EB0], ymm3
       vmulps   ymm3, ymm4, ymmword ptr [rbp-0x810]
       vmovups  ymmword ptr [rbp-0x1ED0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1D70]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x810]
       vmovups  ymmword ptr [rbp-0x1EF0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1D30]
       vaddps   ymm14, ymm3, ymm14
       vmovups  ymmword ptr [rbp-0x1E90], ymm14
       vmovups  ymm3, ymmword ptr [rbp-0x1D50]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x1EB0]
       vmovups  ymmword ptr [rbp-0x1EB0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1D30]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x1ED0]
       vmovups  ymmword ptr [rbp-0x1ED0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1D50]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x1EF0]
       vmovups  ymmword ptr [rbp-0x1EF0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x5D0]
       vsubps   ymm3, ymm3, ymmword ptr [rbp-0x590]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x5B0]
       vminps   ymm3, ymm8, ymm3
       vxorps   ymm14, ymm14, ymm14
       vcmpeqps ymm14, ymm14, ymm3
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymm3
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vcmpltps ymm14, ymm3, ymm14
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vandnps  ymm3, ymm14, ymm3
       vmovups  ymmword ptr [rbp-0x830], ymm3
       vmovups  ymm14, ymmword ptr [rbp-0x5F0]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x590]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x5B0]
       vminps   ymm14, ymm8, ymm14
 
G_M000_IG26:                ;; offset=0x2568
       vxorps   ymm3, ymm3, ymm3
       vcmpeqps ymm3, ymm3, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm3, ymm3, ymm14
       vandps   ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vxorps   ymm3, ymm3, ymm3
       vcmpltps ymm3, ymm14, ymm3
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vandnps  ymm3, ymm3, ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x830]
       vmulps   ymm3, ymm14, ymm3
       vsubps   ymm14, ymm8, ymm3
       vmovups  ymmword ptr [rbp-0x850], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1830]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x1D30]
       vmovups  ymmword ptr [rbp-0x1F10], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1850]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x1D50]
       vmovups  ymmword ptr [rbp-0x1F30], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1B50]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1F10]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmulps   ymm14, ymm4, ymmword ptr [rbp-0x1F30]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x870], ymm14
       vmulps   ymm4, ymm4, ymmword ptr [rbp-0x1F10]
       vmovups  ymm14, ymmword ptr [rbp-0x1D70]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1F30]
       vaddps   ymm4, ymm14, ymm4
       vmovups  ymmword ptr [rbp-0x890], ymm4
       vmovups  ymm14, ymmword ptr [rbp-0x3070]
       vandps   ymm4, ymm14, ymmword ptr [rbp-0x870]
       vmovups  ymmword ptr [rbp-0x3310], ymm4
       vandps   ymm4, ymm14, ymmword ptr [rbp-0x890]
       vmovups  ymmword ptr [rbp-0x3330], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x3310]
       vcmpgtps ymm4, ymm4, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x8B0], ymm4
       vxorps   ymm4, ymm4, ymm4
       vmovups  ymmword ptr [rbp-0x3330], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x870]
       vcmpgtps ymm4, ymm4, ymmword ptr [rbp-0x3330]
       vpand    ymm4, ymm4, ymmword ptr [rbp-0x8B0]
       vpor     ymm4, ymm4, ymmword ptr [rbp-0x650]
       vmovups  ymmword ptr [rbp-0x8D0], ymm4
       vxorps   ymm4, ymm4, ymm4
       vmovups  ymmword ptr [rbp-0x3330], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x870]
       vcmpleps ymm4, ymm4, ymmword ptr [rbp-0x3330]
       vpand    ymm4, ymm4, ymmword ptr [rbp-0x8B0]
       vmovups  ymmword ptr [rbp-0x8F0], ymm4
       vxorps   ymm4, ymm4, ymm4
       vmovups  ymmword ptr [rbp-0x3330], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x890]
       vcmpltps ymm4, ymm4, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x3330], ymm4
       vpcmpeqd ymm4, ymm4, ymm4
 
G_M000_IG27:                ;; offset=0x2711
       vpxor    ymm4, ymm4, ymmword ptr [rbp-0x8B0]
       vmovups  ymmword ptr [rbp-0x2FB0], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x3330]
       vpand    ymm4, ymm4, ymmword ptr [rbp-0x2FB0]
       vmovups  ymmword ptr [rbp-0x910], ymm4
       vxorps   ymm4, ymm4, ymm4
       vmovups  ymmword ptr [rbp-0x3330], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x890]
       vcmpgeps ymm4, ymm4, ymmword ptr [rbp-0x3330]
       vpand    ymm4, ymm4, ymmword ptr [rbp-0x2FB0]
       vmovups  ymmword ptr [rbp-0x930], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x1830]
       vmulps   ymm14, ymm4, ymmword ptr [rbp-0x850]
       vmovups  ymmword ptr [rbp-0x31D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1AD0]
       vmulps   ymm4, ymm3, ymm14
       vaddps   ymm4, ymm4, ymmword ptr [rbp-0x31D0]
       vandps   ymm4, ymm4, ymmword ptr [rbp-0x8D0]
       vmovups  ymmword ptr [rbp-0x3330], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x8D0]
       vandnps  ymm14, ymm4, ymm14
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x1AD0], ymm14
       vmovups  ymm4, ymmword ptr [rbp-0x1850]
       vmulps   ymm4, ymm4, ymmword ptr [rbp-0x850]
       vmovups  ymmword ptr [rbp-0x31F0], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x1AF0]
       vmulps   ymm14, ymm3, ymm4
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x31F0]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x8D0]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x8D0]
       vandnps  ymm4, ymm14, ymm4
       vorps    ymm4, ymm4, ymmword ptr [rbp-0x3330]
       vmulps   ymm14, ymm3, ymmword ptr [rbp-0x1CF0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x31D0]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x8F0]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x8F0]
       vandnps  ymm14, ymm14, ymmword ptr [rbp-0x1CF0]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x1CF0], ymm14
       vmulps   ymm14, ymm3, ymmword ptr [rbp-0x1D10]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x31F0]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x8F0]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x8F0]
       vandnps  ymm14, ymm14, ymmword ptr [rbp-0x1D10]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x1D10], ymm14
       vmulps   ymm14, ymm3, ymmword ptr [rbp-0x1E90]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x31D0]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x910]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x910]
       vandnps  ymm14, ymm14, ymmword ptr [rbp-0x1E90]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x1E90], ymm14
       vmulps   ymm14, ymm3, ymmword ptr [rbp-0x1EB0]
 
G_M000_IG28:                ;; offset=0x28D8
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x31F0]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x910]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x910]
       vandnps  ymm14, ymm14, ymmword ptr [rbp-0x1EB0]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x1EB0], ymm14
       vmulps   ymm14, ymm3, ymmword ptr [rbp-0x1ED0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x31D0]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x930]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x930]
       vandnps  ymm14, ymm14, ymmword ptr [rbp-0x1ED0]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x1ED0], ymm14
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x1EF0]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x31F0]
       vandps   ymm3, ymm3, ymmword ptr [rbp-0x930]
       vmovups  ymmword ptr [rbp-0x3330], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x930]
       vandnps  ymm3, ymm3, ymmword ptr [rbp-0x1EF0]
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x1EF0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1CF0]
       vmovups  ymmword ptr [rbp-0x1590], ymm3
       vmovaps  ymm3, ymm2
       vmovups  ymm14, ymmword ptr [rbp-0x1D10]
       vmovups  ymmword ptr [rbp-0x15D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1E90]
       vmovups  ymmword ptr [rbp-0x15F0], ymm14
       vmovups  ymmword ptr [rbp-0x1610], ymm3
       vmovups  ymm14, ymmword ptr [rbp-0x1EB0]
       vmovups  ymmword ptr [rbp-0x1630], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1ED0]
       vmovups  ymmword ptr [rbp-0x1650], ymm14
       vmovups  ymmword ptr [rbp-0x1670], ymm3
       vmovups  ymm14, ymmword ptr [rbp-0x1EF0]
       vmovups  ymmword ptr [rbp-0x1690], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x730]
       vcmpgtps ymm14, ymm14, ymmword ptr [rbp-0x710]
       vpand    ymm14, ymm14, ymmword ptr [rbp-0x430]
       vpand    ymm14, ymm14, ymmword ptr [rbp-0x2FD0]
       vmovups  ymmword ptr [rbx+0x300], ymm14
       vmovups  ymm14, ymmword ptr [rbx+0x300]
       vmovups  ymmword ptr [rbx+0x320], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x810]
       vcmpgtps ymm14, ymm14, ymmword ptr [rbp-0x7F0]
       vpand    ymm14, ymm14, ymmword ptr [rbx+0x300]
       vmovups  ymmword ptr [rbx+0x340], ymm14
       vmovups  ymmword ptr [rbp-0x15B0], ymm3
       vmovups  ymmword ptr [rbp-0x1AF0], ymm4
 
G_M000_IG29:                ;; offset=0x2A6E
       vmovups  ymm14, ymmword ptr [rbp-0x1AD0]
       vmovups  ymmword ptr [rbp-0x1530], ymm14
       vmovaps  ymm14, ymm2
       vmovups  ymm4, ymmword ptr [rbp-0x1AF0]
       vmovups  ymmword ptr [rbp-0x1570], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x430]
       vmovups  ymmword ptr [rbx+0x2E0], ymm4
       vmovups  ymmword ptr [rbp-0x1550], ymm14
 
G_M000_IG30:                ;; offset=0x2AAA
       vmovups  ymm4, ymmword ptr [rbp-0x410]
       vpandn   ymm4, ymm4, ymm11
       vmovups  ymmword ptr [rbp-0x3330], ymm4
       vpandn   ymm4, ymm11, ymmword ptr [rbp-0x410]
       vpor     ymm4, ymm4, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x3330], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x2D0]
       vpandn   ymm4, ymm4, ymmword ptr [rbp-0x3330]
       vmovups  ymm14, ymmword ptr [rbp-0x1E0]
       vmovups  ymmword ptr [rbp-0x3210], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1C0]
       vmovups  ymmword ptr [rbp-0x3230], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x280]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1A0]
       vmovups  ymmword ptr [rbp-0x3250], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x260]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x490], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x120]
       vmovups  ymmword ptr [rbp-0x3270], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x100]
       vmovups  ymmword ptr [rbp-0x3290], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x280]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0xE0]
       vmovups  ymmword ptr [rbp-0x32B0], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x260]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x4B0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x490]
       vmulps   ymm14, ymm14, ymm14
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x4B0]
       vmulps   ymm14, ymm14, ymm14
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vsqrtps  ymm14, ymm14
       vdivps   ymm14, ymm8, ymm14
       vmovups  ymmword ptr [rbp-0x4D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x490]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x4D0]
       vmovups  ymmword ptr [rbp-0x4F0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x4B0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x4D0]
       vmovups  ymmword ptr [rbp-0x510], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x4F0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x3210]
       vmovups  ymmword ptr [rbp-0x1930], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x4F0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x3230]
       vmovups  ymmword ptr [rbp-0x1950], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x4F0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x3250]
 
G_M000_IG31:                ;; offset=0x2C73
       vmovups  ymmword ptr [rbp-0x1970], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x510]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x3270]
       vmovups  ymmword ptr [rbp-0x1990], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x510]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x3290]
       vmovups  ymmword ptr [rbp-0x19B0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x510]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x32B0]
       vmovups  ymmword ptr [rbp-0x19D0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1930]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x1990]
       vmovups  ymmword ptr [rbp-0x19F0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1950]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x19B0]
       vmovups  ymmword ptr [rbp-0x1A10], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1970]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x19D0]
       vmovups  ymmword ptr [rbp-0x1A30], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1770]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x240]
       vmovups  ymmword ptr [rbp-0x1A50], ymm14
       vmovups  ymm3, ymmword ptr [rbp-0x1790]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x220]
       vmovups  ymmword ptr [rbp-0x1A70], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x17B0]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x200]
       vmovups  ymmword ptr [rbp-0x1A90], ymm3
       vxorps   ymm3, ymm3, ymm3
       vmovups  ymmword ptr [rbp-0x1AB0], ymm3
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm3, ymm3, ymm4
       vptest   ymm3, ymm3
       je       G_M000_IG39
       vaddps   ymm1, ymm14, ymmword ptr [rbp-0x180]
       vmovups  ymmword ptr [rbp-0x1F50], ymm1
       vmovups  ymm0, ymmword ptr [rbp-0x160]
       vmovups  ymmword ptr [rbp-0x32D0], ymm0
       vaddps   ymm5, ymm0, ymmword ptr [rbp-0x1A70]
       vmovups  ymmword ptr [rbp-0x1F70], ymm5
       vmovups  ymm1, ymmword ptr [rbp-0x140]
       vmovups  ymmword ptr [rbp-0x32F0], ymm1
       vaddps   ymm5, ymm1, ymmword ptr [rbp-0x1A90]
       vmovups  ymmword ptr [rbp-0x1F90], ymm5
       vmovups  ymmword ptr [rbp-0x1FB0], ymm8
       vmovups  ymm5, ymmword ptr [rbp-0x14D0]
       vsubps   ymm1, ymm5, ymmword ptr [rbp-0x1830]
       vmulps   ymm1, ymm1, ymmword ptr [rbp-0x180]
       vmovups  ymmword ptr [rbp-0x2F50], ymm1
       vmulps   ymm1, ymm0, ymmword ptr [rbp-0x2FF0]
       vaddps   ymm1, ymm1, ymmword ptr [rbp-0x2F50]
       vmovups  ymmword ptr [rbp-0x3330], ymm1
       vmovups  ymm0, ymmword ptr [rbp-0x1510]
       vmovups  ymm1, ymmword ptr [rbp-0x1850]
       vsubps   ymm3, ymm0, ymm1
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x32F0]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x11D0], ymm3
 
G_M000_IG32:                ;; offset=0x2E2E
       vmovups  ymm3, ymmword ptr [rbp-0x3B0]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x11D0]
       vmovups  ymmword ptr [rbp-0x11F0], ymm3
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x2A10], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x11F0]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x280]
       vmovups  ymmword ptr [rbp-0x2A30], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x11F0]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x260]
       vmovups  ymmword ptr [rbp-0x2A50], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1830]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x2A10]
       vmovups  ymmword ptr [rbp-0x2A70], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1AB0]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x2A30]
       vmovups  ymmword ptr [rbp-0x2A90], ymm3
       vaddps   ymm3, ymm1, ymmword ptr [rbp-0x2A50]
       vmovups  ymmword ptr [rbp-0x2AB0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x2A70]
       vsubps   ymm3, ymm3, ymm5
       vmovups  ymmword ptr [rbp-0x2AD0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x2A90]
       vmovups  ymm1, ymmword ptr [rbp-0x14F0]
       vsubps   ymm3, ymm3, ymm1
       vmovups  ymmword ptr [rbp-0x2AF0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x2AB0]
       vsubps   ymm3, ymm3, ymm0
       vmovups  ymmword ptr [rbp-0x2B10], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x3210]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x2AD0]
       vmovups  ymmword ptr [rbp-0x3330], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x3230]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x2AF0]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x3330], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x3250]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x2B10]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x1FD0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x3270]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x2AD0]
       vmovups  ymmword ptr [rbp-0x3330], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x3290]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x2AF0]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x3330], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x32B0]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x2B10]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x1FF0], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1A70]
       vsubps   ymm3, ymm3, ymm2
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x3D0]
       vmovups  ymmword ptr [rbp-0x1210], ymm3
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x2A0]
       vsubps   ymm3, ymm14, ymm3
       vmovups  ymmword ptr [rbp-0x2010], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1210]
 
G_M000_IG33:                ;; offset=0x2FF2
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x260]
       vmovups  ymmword ptr [rbp-0x3330], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1A90]
       vsubps   ymm3, ymm3, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x2030], ymm3
       vsubps   ymm3, ymm1, ymmword ptr [rbp-0x1FB0]
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x32D0]
       vaddps   ymm3, ymm3, ymmword ptr [rbp-0x2F50]
       vmovups  ymmword ptr [rbp-0x3330], ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1850]
       vsubps   ymm14, ymm0, ymm3
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x32F0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x1230], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3B0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1230]
       vmovups  ymmword ptr [rbp-0x1250], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x2B30], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1250]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x280]
       vmovups  ymmword ptr [rbp-0x2B50], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1250]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x260]
       vmovups  ymmword ptr [rbp-0x2B70], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1830]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x2B30]
       vmovups  ymmword ptr [rbp-0x2B90], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1FB0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x2B50]
       vmovups  ymmword ptr [rbp-0x2BB0], ymm14
       vaddps   ymm14, ymm3, ymmword ptr [rbp-0x2B70]
       vmovups  ymmword ptr [rbp-0x2BD0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2B90]
       vmovups  ymmword ptr [rbp-0x14D0], ymm5
       vsubps   ymm14, ymm14, ymm5
       vmovups  ymmword ptr [rbp-0x2BF0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2BB0]
       vmovups  ymmword ptr [rbp-0x14F0], ymm1
       vsubps   ymm14, ymm14, ymm1
       vmovups  ymmword ptr [rbp-0x2C10], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2BD0]
       vmovups  ymmword ptr [rbp-0x1510], ymm0
       vsubps   ymm14, ymm14, ymm0
       vmovups  ymmword ptr [rbp-0x2C30], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3210]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2BF0]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3230]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2C10]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3250]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2C30]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x2050], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x3270]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2BF0]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
 
G_M000_IG34:                ;; offset=0x31BA
       vmovups  ymm14, ymmword ptr [rbp-0x3290]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2C10]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x32B0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2C30]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x2070], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1F70]
       vsubps   ymm14, ymm14, ymm2
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x3D0]
       vmovups  ymmword ptr [rbp-0x1270], ymm14
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1F50]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x2090], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1270]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x260]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1F90]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x20B0], ymm14
       vandps   ymm14, ymm11, ymmword ptr [rbp-0x1FD0]
       vmovups  ymmword ptr [rbp-0x3330], ymm14
       vandnps  ymm14, ymm11, ymmword ptr [rbp-0x2010]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x20D0], ymm14
       vandps   ymm3, ymm11, ymmword ptr [rbp-0x1FF0]
       vmovups  ymmword ptr [rbp-0x3330], ymm3
       vandnps  ymm3, ymm11, ymmword ptr [rbp-0x2030]
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x20F0], ymm3
       vandps   ymm1, ymm11, ymmword ptr [rbp-0x2050]
       vmovups  ymmword ptr [rbp-0x3330], ymm1
       vandnps  ymm1, ymm11, ymmword ptr [rbp-0x2090]
       vorps    ymm1, ymm1, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x2110], ymm1
       vandps   ymm1, ymm11, ymmword ptr [rbp-0x2070]
       vmovups  ymmword ptr [rbp-0x3330], ymm1
       vandnps  ymm1, ymm11, ymmword ptr [rbp-0x20B0]
       vorps    ymm1, ymm1, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x2130], ymm1
       vandps   ymm1, ymm11, ymmword ptr [rsi]
       vmovups  ymmword ptr [rbp-0x3330], ymm1
       vandnps  ymm1, ymm11, ymmword ptr [rdi]
       vorps    ymm1, ymm1, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x950], ymm1
       vandps   ymm1, ymm11, ymmword ptr [rdi+0x20]
       vmovups  ymmword ptr [rbp-0x3330], ymm1
       vandnps  ymm1, ymm11, ymmword ptr [rsi+0x20]
       vorps    ymm1, ymm1, ymmword ptr [rbp-0x3330]
       vmovups  ymm5, ymmword ptr [rbp-0x2110]
       vsubps   ymm5, ymm5, ymm14
       vmovups  ymmword ptr [rbp-0x2150], ymm5
       vmovups  ymm5, ymmword ptr [rbp-0x2130]
       vsubps   ymm5, ymm5, ymm3
       vmovups  ymmword ptr [rbp-0x2170], ymm5
       vmovups  ymm5, ymmword ptr [rbp-0x2150]
 
G_M000_IG35:                ;; offset=0x3379
       vmulps   ymm5, ymm5, ymm5
       vmovups  ymmword ptr [rbp-0x3330], ymm5
       vmovups  ymm5, ymmword ptr [rbp-0x2170]
       vmulps   ymm5, ymm5, ymm5
       vaddps   ymm5, ymm5, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x1290], ymm5
       vdivps   ymm5, ymm8, ymm5
       vmovups  ymmword ptr [rbp-0x12B0], ymm5
       vmulps   ymm5, ymm14, ymmword ptr [rbp-0x2150]
       vmovups  ymmword ptr [rbp-0x3330], ymm5
       vmulps   ymm5, ymm3, ymmword ptr [rbp-0x2170]
       vaddps   ymm5, ymm5, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x12D0], ymm5
       vmulps   ymm5, ymm14, ymm14
       vmovups  ymmword ptr [rbp-0x3330], ymm5
       vmulps   ymm5, ymm3, ymm3
       vaddps   ymm5, ymm5, ymmword ptr [rbp-0x3330]
       vmovups  ymmword ptr [rbp-0x3330], ymm5
       vmovups  ymm5, ymmword ptr [rbp-0x950]
       vmulps   ymm5, ymm5, ymm5
       vmovups  ymmword ptr [rbp-0x3310], ymm5
       vmovups  ymm5, ymmword ptr [rbp-0x3330]
       vsubps   ymm5, ymm5, ymmword ptr [rbp-0x3310]
       vmulps   ymm5, ymm5, ymmword ptr [rbp-0x1290]
       vmovups  ymmword ptr [rbp-0x3310], ymm5
       vmovups  ymm5, ymmword ptr [rbp-0x12D0]
       vmulps   ymm5, ymm5, ymm5
       vsubps   ymm5, ymm5, ymmword ptr [rbp-0x3310]
       vxorps   ymm3, ymm3, ymm3
       vcmpeqps ymm3, ymm3, ymm5
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm3, ymm3, ymm5
       vandps   ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vxorps   ymm3, ymm3, ymm3
       vcmpltps ymm3, ymm5, ymm3
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vandnps  ymm3, ymm3, ymm5
       vsqrtps  ymm3, ymm3
       vmulps   ymm3, ymm3, ymmword ptr [rbp-0x12B0]
       vmovups  ymmword ptr [rbp-0x12F0], ymm3
       vxorps   ymm5, ymm9, ymmword ptr [rbp-0x12D0]
       vmulps   ymm5, ymm5, ymmword ptr [rbp-0x12B0]
       vmovups  ymmword ptr [rbp-0x1310], ymm5
       vsubps   ymm3, ymm5, ymm3
       vaddps   ymm5, ymm5, ymmword ptr [rbp-0x12F0]
       vmovups  ymm14, ymmword ptr [rbp-0x3070]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x1290]
       vcmpltps ymm14, ymm14, ymmword ptr [reloc @RWD224]
       vmovups  ymmword ptr [rbp-0x1330], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1310]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x1330]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1330]
       vandnps  ymm3, ymm14, ymm3
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymm14, ymmword ptr [rbp-0x1310]
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x1330]
 
G_M000_IG36:                ;; offset=0x3511
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1330]
       vandnps  ymm5, ymm14, ymm5
       vorps    ymm5, ymm5, ymmword ptr [rbp-0x3310]
       vxorps   ymm14, ymm9, ymm1
       vcmpeqps ymm0, ymm3, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm0
       vxorps   ymm0, ymm0, ymm0
       vpcmpgtd ymm0, ymm0, ymm3
       vandps   ymm0, ymm0, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm0
       vcmpneqps ymm0, ymm14, ymm14
       vorps    ymm0, ymm0, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm0
       vcmpltps ymm0, ymm3, ymm14
       vorps    ymm0, ymm0, ymmword ptr [rbp-0x3310]
       vblendvps ymm3, ymm3, ymm14, ymm0
       vcmpeqps ymm0, ymm1, ymm3
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymm1
       vandps   ymm0, ymm14, ymm0
       vcmpneqps ymm14, ymm1, ymm1
       vorps    ymm0, ymm14, ymm0
       vcmpltps ymm14, ymm1, ymm3
       vorps    ymm0, ymm14, ymm0
       vblendvps ymm3, ymm3, ymm1, ymm0
       vcmpeqps ymm0, ymm1, ymm5
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymm1
       vandps   ymm0, ymm14, ymm0
       vcmpneqps ymm14, ymm1, ymm1
       vorps    ymm0, ymm14, ymm0
       vcmpltps ymm14, ymm1, ymm5
       vorps    ymm0, ymm14, ymm0
       vblendvps ymm5, ymm5, ymm1, ymm0
       vmulps   ymm1, ymm3, ymmword ptr [rbp-0x2150]
       vmovups  ymm14, ymmword ptr [rbp-0x20D0]
       vaddps   ymm1, ymm1, ymm14
       vmovups  ymmword ptr [rbp-0x2190], ymm1
       vmulps   ymm1, ymm3, ymmword ptr [rbp-0x2170]
       vmovups  ymm0, ymmword ptr [rbp-0x20F0]
       vaddps   ymm1, ymm1, ymm0
       vmovups  ymmword ptr [rbp-0x21B0], ymm1
       vmulps   ymm1, ymm5, ymmword ptr [rbp-0x2150]
       vaddps   ymm1, ymm1, ymm14
       vmovups  ymmword ptr [rbp-0x21D0], ymm1
       vmulps   ymm14, ymm5, ymmword ptr [rbp-0x2170]
       vaddps   ymm0, ymm14, ymm0
       vmovups  ymmword ptr [rbp-0x21F0], ymm0
       vmovups  ymm14, ymmword ptr [rbp-0x1830]
       vandps   ymm0, ymm11, ymm14
       vandnps  ymm1, ymm11, ymmword ptr [rbp-0x2190]
       vorps    ymm0, ymm1, ymm0
       vmovups  ymmword ptr [rbp-0x2210], ymm0
       vandps   ymm1, ymm11, ymm3
       vandnps  ymm0, ymm11, ymm2
       vorps    ymm0, ymm0, ymm1
       vmovups  ymmword ptr [rbp-0x2230], ymm0
       vmovups  ymm1, ymmword ptr [rbp-0x1850]
 
G_M000_IG37:                ;; offset=0x3674
       vandps   ymm0, ymm11, ymm1
       vmovups  ymmword ptr [rbp-0x3310], ymm0
       vandnps  ymm0, ymm11, ymmword ptr [rbp-0x21B0]
       vorps    ymm0, ymm0, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x2250], ymm0
       vandps   ymm0, ymm11, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm0
       vandnps  ymm0, ymm11, ymmword ptr [rbp-0x21D0]
       vorps    ymm0, ymm0, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x2270], ymm0
       vandps   ymm0, ymm11, ymm5
       vandnps  ymm2, ymm11, ymm2
       vorps    ymm0, ymm2, ymm0
       vmovups  ymmword ptr [rbp-0x2290], ymm0
       vandps   ymm2, ymm11, ymm1
       vandnps  ymm0, ymm11, ymmword ptr [rbp-0x21F0]
       vorps    ymm0, ymm0, ymm2
       vmovups  ymmword ptr [rbp-0x22B0], ymm0
       vandps   ymm2, ymm4, ymmword ptr [rbp-0x2210]
       vandnps  ymm0, ymm4, ymmword ptr [rbp-0x1530]
       vorps    ymm0, ymm0, ymm2
       vmovups  ymmword ptr [rbp-0x1530], ymm0
       vandps   ymm2, ymm4, ymmword ptr [rbp-0x2230]
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vandnps  ymm2, ymm4, ymmword ptr [rbp-0x1550]
       vorps    ymm2, ymm2, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1550], ymm2
       vandps   ymm2, ymm4, ymmword ptr [rbp-0x2250]
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vandnps  ymm2, ymm4, ymmword ptr [rbp-0x1570]
       vorps    ymm2, ymm2, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1570], ymm2
       vandps   ymm2, ymm4, ymmword ptr [rbp-0x2270]
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vandnps  ymm2, ymm4, ymmword ptr [rbp-0x1590]
       vorps    ymm2, ymm2, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1590], ymm2
       vandps   ymm2, ymm4, ymmword ptr [rbp-0x2290]
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vandnps  ymm2, ymm4, ymmword ptr [rbp-0x15B0]
       vorps    ymm2, ymm2, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x15B0], ymm2
       vandps   ymm0, ymm4, ymmword ptr [rbp-0x22B0]
       vmovups  ymmword ptr [rbp-0x3310], ymm0
       vandnps  ymm0, ymm4, ymmword ptr [rbp-0x15D0]
       vorps    ymm0, ymm0, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x15D0], ymm0
       vpcmpeqd ymm0, ymm0, ymm0
       vpand    ymm0, ymm0, ymm4
       vmovups  ymmword ptr [rbp-0x3310], ymm0
       vpandn   ymm0, ymm4, ymmword ptr [rbx+0x2E0]
       vpor     ymm0, ymm0, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbx+0x2E0], ymm0
       vcmpgtps ymm0, ymm5, ymm3
       vpand    ymm0, ymm0, ymm4
       vpandn   ymm3, ymm4, ymmword ptr [rbx+0x300]
       vpor     ymm0, ymm3, ymm0
       vmovups  ymmword ptr [rbx+0x300], ymm0
       vandps   ymm3, ymm11, ymmword ptr [rbp-0x1870]
 
G_M000_IG38:                ;; offset=0x381A
       vandnps  ymm5, ymm11, ymmword ptr [rbp-0x19F0]
       vorps    ymm3, ymm5, ymm3
       vandps   ymm5, ymm11, ymmword ptr [rbp-0x1890]
       vandnps  ymm0, ymm11, ymmword ptr [rbp-0x1A10]
       vorps    ymm0, ymm0, ymm5
       vmovups  ymmword ptr [rbp-0x22D0], ymm0
       vandps   ymm5, ymm11, ymmword ptr [rbp-0x18B0]
       vandnps  ymm0, ymm11, ymmword ptr [rbp-0x1A30]
       vorps    ymm0, ymm0, ymm5
       vmovups  ymmword ptr [rbp-0x22F0], ymm0
       vandps   ymm3, ymm4, ymm3
       vandnps  ymm5, ymm4, ymmword ptr [rbp-0x1870]
       vorps    ymm3, ymm5, ymm3
       vmovups  ymmword ptr [rbp-0x18D0], ymm3
       vandps   ymm5, ymm4, ymmword ptr [rbp-0x22D0]
       vandnps  ymm0, ymm4, ymmword ptr [rbp-0x1890]
       vorps    ymm0, ymm0, ymm5
       vmovups  ymmword ptr [rbp-0x18F0], ymm0
       vandps   ymm5, ymm4, ymmword ptr [rbp-0x22F0]
       vmovups  ymmword ptr [rbp-0x3310], ymm5
       vandnps  ymm5, ymm4, ymmword ptr [rbp-0x18B0]
       vorps    ymm5, ymm5, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1910], ymm5
       vmovups  ymm14, ymmword ptr [rbp-0x14D0]
       vandps   ymm1, ymm11, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm1
       vandnps  ymm1, ymm11, ymmword ptr [rbp-0x1A50]
       vorps    ymm1, ymm1, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x2310], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x14F0]
       vandps   ymm5, ymm11, ymm1
       vmovups  ymmword ptr [rbp-0x3310], ymm5
       vandnps  ymm5, ymm11, ymmword ptr [rbp-0x1A70]
       vorps    ymm5, ymm5, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x2330], ymm5
       vmovups  ymm5, ymmword ptr [rbp-0x1510]
       vandps   ymm0, ymm11, ymm5
       vmovups  ymmword ptr [rbp-0x3310], ymm0
       vandnps  ymm0, ymm11, ymmword ptr [rbp-0x1A90]
       vorps    ymm0, ymm0, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x2350], ymm0
       vandps   ymm0, ymm4, ymmword ptr [rbp-0x2310]
       vandnps  ymm14, ymm4, ymm14
       vorps    ymm0, ymm14, ymm0
       vandps   ymm14, ymm4, ymmword ptr [rbp-0x2330]
       vandnps  ymm1, ymm4, ymm1
       vorps    ymm1, ymm1, ymm14
       vandps   ymm14, ymm4, ymmword ptr [rbp-0x2350]
       vandnps  ymm4, ymm4, ymm5
       vorps    ymm5, ymm4, ymm14
       vmovaps  ymm2, ymm0
       vmovaps  ymm0, ymm1
       vmovaps  ymm1, ymm2
       vmovups  ymm14, ymmword ptr [rbp-0x1A50]
 
G_M000_IG39:                ;; offset=0x3986
       vpcmpeqd ymm4, ymm4, ymm4
       vpxor    ymm4, ymm4, ymmword ptr [rbp-0x410]
       vpandn   ymm4, ymm11, ymm4
       vmovups  ymm11, ymmword ptr [rbp-0x2D0]
       vpandn   ymm4, ymm11, ymm4
       vxorps   ymm11, ymm11, ymm11
       vpcmpgtd ymm11, ymm11, ymm4
       vptest   ymm11, ymm11
       je       G_M000_IG44
       vxorps   ymm11, ymm9, ymm14
       vmovups  ymmword ptr [rbp-0x2370], ymm11
       vxorps   ymm11, ymm9, ymmword ptr [rbp-0x1A70]
       vmovups  ymmword ptr [rbp-0x2390], ymm11
       vxorps   ymm14, ymm9, ymmword ptr [rbp-0x1A90]
       vmovups  ymmword ptr [rbp-0x23B0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x2A0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2A0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x260]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x260]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vdivps   ymm14, ymm8, ymm14
       vmovups  ymmword ptr [rbp-0x970], ymm14
       add      rsi, 32
       add      rdi, 32
       vmovups  ymm14, ymmword ptr [rbp-0x2370]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x180]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmulps   ymm14, ymm11, ymmword ptr [rbp-0x160]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x23B0]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x140]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1390], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x160]
       vmovups  ymmword ptr [rbp-0x13B0], ymm14
       vmulps   ymm11, ymm11, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm11
       vmovups  ymm11, ymmword ptr [rbp-0x1390]
       vsubps   ymm11, ymm11, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x13D0], ymm11
       vbroadcastss ymm11, dword ptr [reloc @RWD288]
       vmulps   ymm14, ymm14, ymm14
       vsubps   ymm14, ymm8, ymm14
       vcmpeqps ymm3, ymm11, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vxorps   ymm3, ymm3, ymm3
       vpcmpgtd ymm3, ymm3, ymm14
       vandps   ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vcmpltps ymm3, ymm14, ymm11
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vblendvps ymm3, ymm14, ymm11, ymm3
       vmovups  ymm11, ymmword ptr [rbp-0x13D0]
       vdivps   ymm3, ymm11, ymm3
       vmovups  ymm14, ymmword ptr [rbp-0x13B0]
 
G_M000_IG40:                ;; offset=0x3B0F
       vmulps   ymm3, ymm14, ymm3
       vsubps   ymm3, ymm3, ymmword ptr [rbp-0x2390]
       vmovups  ymmword ptr [rbp-0x1350], ymm3
       vmovups  ymm11, ymmword ptr [rbp-0x3070]
       vandps   ymm11, ymm11, ymm14
       vmulps   ymm11, ymm11, ymmword ptr [rsi]
       vmovups  ymmword ptr [rbp-0x13F0], ymm11
       vxorps   ymm14, ymm9, ymmword ptr [rdi]
       vmovups  ymm11, ymmword ptr [rdi]
       vxorps   ymm2, ymm9, ymmword ptr [rbp-0x13F0]
       vsubps   ymm2, ymm2, ymmword ptr [rbp-0x2390]
       vmovups  ymmword ptr [rbp-0x1410], ymm2
       vcmpeqps ymm2, ymm11, ymm2
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vxorps   ymm2, ymm2, ymm2
       vpcmpgtd ymm2, ymm2, ymm11
       vandps   ymm2, ymm2, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vcmpneqps ymm2, ymm11, ymm11
       vorps    ymm2, ymm2, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vcmpltps ymm2, ymm11, ymmword ptr [rbp-0x1410]
       vorps    ymm2, ymm2, ymmword ptr [rbp-0x3310]
       vmovups  ymm3, ymmword ptr [rbp-0x1410]
       vblendvps ymm2, ymm3, ymm11, ymm2
       vcmpeqps ymm3, ymm14, ymm2
       vxorps   ymm11, ymm11, ymm11
       vpcmpgtd ymm11, ymm11, ymm2
       vandps   ymm3, ymm11, ymm3
       vcmpneqps ymm11, ymm14, ymm14
       vorps    ymm3, ymm11, ymm3
       vcmpltps ymm11, ymm2, ymm14
       vorps    ymm3, ymm11, ymm3
       vblendvps ymm2, ymm2, ymm14, ymm3
       vmovups  ymmword ptr [rbp-0x1370], ymm2
       vmovups  ymm3, ymmword ptr [rdi]
       vxorps   ymm11, ymm9, ymmword ptr [rdi]
       vmovups  ymm14, ymmword ptr [rbp-0x13F0]
       vsubps   ymm14, ymm14, ymmword ptr [rbp-0x2390]
       vcmpeqps ymm2, ymm11, ymm14
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vxorps   ymm2, ymm2, ymm2
       vpcmpgtd ymm2, ymm2, ymm14
       vandps   ymm2, ymm2, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vcmpneqps ymm2, ymm11, ymm11
       vorps    ymm2, ymm2, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vcmpltps ymm2, ymm14, ymm11
       vorps    ymm2, ymm2, ymmword ptr [rbp-0x3310]
       vblendvps ymm2, ymm14, ymm11, ymm2
       vcmpeqps ymm11, ymm3, ymm2
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymm3
       vandps   ymm11, ymm14, ymm11
       vcmpneqps ymm14, ymm3, ymm3
       vorps    ymm11, ymm14, ymm11
       vcmpltps ymm14, ymm3, ymm2
       vorps    ymm11, ymm14, ymm11
 
G_M000_IG41:                ;; offset=0x3C7B
       vblendvps ymm2, ymm2, ymm3, ymm11
       vmovups  ymm3, ymmword ptr [rbp-0x1350]
       vcmpeqps ymm14, ymm3, ymmword ptr [rbp-0x1370]
       vxorps   ymm11, ymm11, ymm11
       vpcmpgtd ymm11, ymm11, ymmword ptr [rbp-0x1370]
       vandps   ymm11, ymm11, ymm14
       vcmpneqps ymm14, ymm3, ymm3
       vorps    ymm11, ymm14, ymm11
       vmovups  ymm14, ymmword ptr [rbp-0x1370]
       vcmpltps ymm14, ymm14, ymm3
       vorps    ymm11, ymm14, ymm11
       vmovups  ymm14, ymmword ptr [rbp-0x1370]
       vblendvps ymm3, ymm14, ymm3, ymm11
       vcmpeqps ymm11, ymm3, ymm2
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymm3
       vandps   ymm11, ymm14, ymm11
       vcmpneqps ymm14, ymm3, ymm3
       vorps    ymm11, ymm14, ymm11
       vcmpltps ymm14, ymm3, ymm2
       vorps    ymm11, ymm14, ymm11
       vblendvps ymm3, ymm2, ymm3, ymm11
       vmovups  ymm11, ymmword ptr [rbp-0x180]
       vmulps   ymm11, ymm11, ymmword ptr [rbp-0x260]
       vmovups  ymm14, ymmword ptr [rbp-0x140]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x2A0]
       vsubps   ymm11, ymm11, ymm14
       vmulps   ymm11, ymm11, ymm11
       vmulps   ymm11, ymm11, ymmword ptr [rbp-0x970]
       vbroadcastss ymm14, dword ptr [reloc @RWD292]
       vsubps   ymm11, ymm14, ymm11
       vmulps   ymm11, ymm11, ymmword ptr [reloc @RWD320]
       vminps   ymm11, ymm8, ymm11
       vxorps   ymm14, ymm14, ymm14
       vcmpeqps ymm14, ymm14, ymm11
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vpcmpgtd ymm14, ymm14, ymm11
       vandps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vxorps   ymm14, ymm14, ymm14
       vcmpltps ymm14, ymm11, ymm14
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vandnps  ymm11, ymm14, ymm11
       vmulps   ymm14, ymm3, ymm11
       vsubps   ymm3, ymm3, ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1370]
       vmulps   ymm14, ymm14, ymm11
       vaddps   ymm14, ymm14, ymm3
       vmovups  ymmword ptr [rbp-0x990], ymm14
       vmulps   ymm2, ymm2, ymm11
       vaddps   ymm2, ymm2, ymm3
       vmovups  ymmword ptr [rbp-0x9B0], ymm2
       vmovups  ymm3, ymmword ptr [rbp-0x1830]
       vandps   ymm11, ymm3, ymm4
       vandnps  ymm2, ymm4, ymmword ptr [rbp-0x1530]
       vorps    ymm2, ymm2, ymm11
       vmovups  ymmword ptr [rbp-0x1530], ymm2
       vandps   ymm11, ymm4, ymm14
 
G_M000_IG42:                ;; offset=0x3DE7
       vandnps  ymm14, ymm4, ymmword ptr [rbp-0x1550]
       vorps    ymm11, ymm14, ymm11
       vmovups  ymmword ptr [rbp-0x1550], ymm11
       vmovups  ymm14, ymmword ptr [rbp-0x1850]
       vandps   ymm11, ymm14, ymm4
       vmovups  ymmword ptr [rbp-0x3310], ymm11
       vandnps  ymm11, ymm4, ymmword ptr [rbp-0x1570]
       vorps    ymm11, ymm11, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1570], ymm11
       vandps   ymm3, ymm3, ymm4
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vandnps  ymm3, ymm4, ymmword ptr [rbp-0x1590]
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1590], ymm3
       vandps   ymm3, ymm4, ymmword ptr [rbp-0x9B0]
       vmovups  ymmword ptr [rbp-0x3310], ymm3
       vandnps  ymm3, ymm4, ymmword ptr [rbp-0x15B0]
       vorps    ymm3, ymm3, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x15B0], ymm3
       vandps   ymm14, ymm14, ymm4
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vandnps  ymm14, ymm4, ymmword ptr [rbp-0x15D0]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x15D0], ymm14
       vpcmpeqd ymm14, ymm14, ymm14
       vpand    ymm14, ymm14, ymm4
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vpandn   ymm14, ymm4, ymmword ptr [rbx+0x2E0]
       vpor     ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbx+0x2E0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x9B0]
       vcmpgtps ymm14, ymm14, ymmword ptr [rbp-0x990]
       vpand    ymm14, ymm14, ymm4
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vpandn   ymm14, ymm4, ymmword ptr [rbx+0x300]
       vpor     ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbx+0x300], ymm14
       vandps   ymm14, ymm4, ymmword ptr [rbp-0x19F0]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vandnps  ymm14, ymm4, ymmword ptr [rbp-0x18D0]
       vorps    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vandps   ymm11, ymm4, ymmword ptr [rbp-0x1A10]
       vmovups  ymmword ptr [rbp-0x3310], ymm11
       vandnps  ymm11, ymm4, ymmword ptr [rbp-0x18F0]
       vorps    ymm11, ymm11, ymmword ptr [rbp-0x3310]
       vandps   ymm2, ymm4, ymmword ptr [rbp-0x1A30]
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vandnps  ymm2, ymm4, ymmword ptr [rbp-0x1910]
       vorps    ymm2, ymm2, ymmword ptr [rbp-0x3310]
       vandps   ymm3, ymm4, ymmword ptr [rbp-0x1A50]
       vandnps  ymm1, ymm4, ymm1
       vorps    ymm1, ymm1, ymm3
       vandps   ymm3, ymm4, ymmword ptr [rbp-0x1A70]
       vandnps  ymm0, ymm4, ymm0
       vorps    ymm0, ymm0, ymm3
       vandps   ymm3, ymm4, ymmword ptr [rbp-0x1A90]
       vandnps  ymm5, ymm4, ymm5
       vorps    ymm5, ymm5, ymm3
       vmovups  ymmword ptr [rbp-0x18D0], ymm14
 
G_M000_IG43:                ;; offset=0x3F8E
       vmovups  ymmword ptr [rbp-0x18F0], ymm11
       vmovups  ymmword ptr [rbp-0x1910], ymm2
 
G_M000_IG44:                ;; offset=0x3F9E
       vmovups  ymm3, ymmword ptr [rbp-0x18D0]
       vmulps   ymm4, ymm3, ymmword ptr [rbp-0x2A0]
       vmovups  ymm11, ymmword ptr [rbp-0x18F0]
       vmulps   ymm14, ymm11, ymmword ptr [rbp-0x280]
       vaddps   ymm4, ymm14, ymm4
       vmovups  ymmword ptr [rbp-0x3310], ymm4
       vmovups  ymm14, ymmword ptr [rbp-0x1910]
       vmulps   ymm4, ymm14, ymmword ptr [rbp-0x260]
       vaddps   ymm4, ymm4, ymmword ptr [rbp-0x3310]
       vdivps   ymm4, ymm8, ymm4
       vmovups  ymmword ptr [rbp-0x530], ymm4
       vxorps   ymm8, ymm9, ymmword ptr [r14]
       vmovups  ymmword ptr [rbp-0x550], ymm8
       lea      rcx, bword ptr [rbx+0x1E0]
       lea      rdx, bword ptr [rbx+0x2E0]
       vmovups  ymm9, ymmword ptr [rbp-0x1530]
       vsubps   ymm8, ymm9, ymm1
       vmulps   ymm8, ymm8, ymm3
       vmovups  ymmword ptr [rbp-0x3310], ymm8
       vmovups  ymm8, ymmword ptr [rbp-0x1550]
       vsubps   ymm8, ymm8, ymm0
       vmulps   ymm8, ymm8, ymm11
       vaddps   ymm8, ymm8, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm8
       vmovups  ymm8, ymmword ptr [rbp-0x1570]
       vsubps   ymm8, ymm8, ymm5
       vmovups  ymmword ptr [rbp-0x1910], ymm14
       vmulps   ymm8, ymm8, ymm14
       vaddps   ymm8, ymm8, ymmword ptr [rbp-0x3310]
       vmulps   ymm8, ymm4, ymm8
       vmovups  ymmword ptr [rcx], ymm8
       vmovups  ymm8, ymmword ptr [rbp-0x1470]
       vaddps   ymm9, ymm8, ymm9
       vmovups  ymmword ptr [rbp-0x2C50], ymm9
       vmovups  ymm8, ymmword ptr [rbp-0x1490]
       vaddps   ymm8, ymm8, ymmword ptr [rbp-0x1550]
       vmovups  ymmword ptr [rbp-0x2C70], ymm8
       vmovups  ymm8, ymmword ptr [rbp-0x14B0]
       vaddps   ymm8, ymm8, ymmword ptr [rbp-0x1570]
       vmovups  ymmword ptr [rbp-0x2C90], ymm8
       vmulps   ymm9, ymm15, ymm9
       vmovups  ymmword ptr [rbp-0x3310], ymm9
       vmovups  ymm8, ymmword ptr [rbp-0x2E50]
       vmulps   ymm9, ymm8, ymmword ptr [rbp-0x2C70]
       vaddps   ymm9, ymm9, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm9
       vmulps   ymm9, ymm13, ymmword ptr [rbp-0x2C90]
       vaddps   ymm9, ymm9, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbx], ymm9
       vmovups  ymm9, ymmword ptr [rbp-0x2E10]
       vmulps   ymm4, ymm9, ymmword ptr [rbp-0x2C50]
       vmovups  ymmword ptr [rbp-0x3310], ymm4
       vmulps   ymm4, ymm7, ymmword ptr [rbp-0x2C70]
       vaddps   ymm4, ymm4, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm4
       vmulps   ymm4, ymm6, ymmword ptr [rbp-0x2C90]
       vaddps   ymm4, ymm4, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbx+0x20], ymm4
       vmovups  ymm4, ymmword ptr [rbp-0x2E30]
 
G_M000_IG45:                ;; offset=0x413F
       vmulps   ymm14, ymm4, ymmword ptr [rbp-0x2C50]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmulps   ymm14, ymm12, ymmword ptr [rbp-0x2C70]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmulps   ymm14, ymm10, ymmword ptr [rbp-0x2C90]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbx+0x40], ymm14
       vmovups  ymm14, ymmword ptr [rdx]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rcx]
       vcmpgeps ymm14, ymm14, ymmword ptr [rbp-0x550]
       vpand    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rdx], ymm14
       lea      rcx, bword ptr [rbx+0x60]
       lea      rdx, bword ptr [rbx+0x200]
       lea      rax, bword ptr [rbx+0x300]
       vmovups  ymm14, ymmword ptr [rbp-0x1590]
       vsubps   ymm14, ymm14, ymm1
       vmulps   ymm14, ymm14, ymm3
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm2, ymmword ptr [rbp-0x15B0]
       vsubps   ymm14, ymm2, ymm0
       vmulps   ymm14, ymm14, ymm11
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x15D0]
       vsubps   ymm14, ymm14, ymm5
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1910]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1430], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x530]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1430]
       vmovups  ymmword ptr [rdx], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1470]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x1590]
       vmovups  ymmword ptr [rbp-0x2CB0], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1490]
       vaddps   ymm2, ymm14, ymm2
       vmovups  ymmword ptr [rbp-0x2CD0], ymm2
       vmovups  ymm14, ymmword ptr [rbp-0x14B0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x15D0]
       vmovups  ymmword ptr [rbp-0x2CF0], ymm14
       vmulps   ymm14, ymm15, ymmword ptr [rbp-0x2CB0]
       vmulps   ymm2, ymm8, ymm2
       vaddps   ymm2, ymm2, ymm14
       vmulps   ymm14, ymm13, ymmword ptr [rbp-0x2CF0]
       vaddps   ymm2, ymm14, ymm2
       vmovups  ymmword ptr [rcx], ymm2
       vmulps   ymm2, ymm9, ymmword ptr [rbp-0x2CB0]
       vmulps   ymm14, ymm7, ymmword ptr [rbp-0x2CD0]
       vaddps   ymm2, ymm14, ymm2
       vmulps   ymm14, ymm6, ymmword ptr [rbp-0x2CF0]
       vaddps   ymm2, ymm14, ymm2
       vmovups  ymmword ptr [rcx+0x20], ymm2
       vmulps   ymm2, ymm4, ymmword ptr [rbp-0x2CB0]
       vmulps   ymm14, ymm12, ymmword ptr [rbp-0x2CD0]
       vaddps   ymm2, ymm14, ymm2
       vmulps   ymm14, ymm10, ymmword ptr [rbp-0x2CF0]
 
G_M000_IG46:                ;; offset=0x42CA
       vaddps   ymm2, ymm14, ymm2
       vmovups  ymmword ptr [rcx+0x40], ymm2
       vmovups  ymm2, ymmword ptr [rax]
       vmovups  ymm14, ymmword ptr [rdx]
       vcmpgeps ymm14, ymm14, ymmword ptr [rbp-0x550]
       vpand    ymm2, ymm14, ymm2
       vmovups  ymmword ptr [rax], ymm2
       lea      rcx, bword ptr [rbx+0xC0]
       lea      rdx, bword ptr [rbx+0x220]
       lea      rax, bword ptr [rbx+0x320]
       vmovups  ymm14, ymmword ptr [rbp-0x15F0]
       vsubps   ymm2, ymm14, ymm1
       vmulps   ymm2, ymm2, ymm3
       vmovups  ymm14, ymmword ptr [rbp-0x1610]
       vsubps   ymm14, ymm14, ymm0
       vmulps   ymm14, ymm14, ymm11
       vaddps   ymm2, ymm14, ymm2
       vmovups  ymmword ptr [rbp-0x3310], ymm2
       vmovups  ymm14, ymmword ptr [rbp-0x1630]
       vsubps   ymm14, ymm14, ymm5
       vmovups  ymm2, ymmword ptr [rbp-0x1910]
       vmulps   ymm14, ymm14, ymm2
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x1450], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x530]
       vmulps   ymm14, ymm14, ymmword ptr [rbp-0x1450]
       vmovups  ymmword ptr [rdx], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1470]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x15F0]
       vmovups  ymmword ptr [rbp-0x2D10], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1490]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x1610]
       vmovups  ymmword ptr [rbp-0x2D30], ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x14B0]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x1630]
       vmovups  ymmword ptr [rbp-0x2D50], ymm14
       vmulps   ymm14, ymm15, ymmword ptr [rbp-0x2D10]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmulps   ymm14, ymm8, ymmword ptr [rbp-0x2D30]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmulps   ymm14, ymm13, ymmword ptr [rbp-0x2D50]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rcx], ymm14
       vmulps   ymm14, ymm9, ymmword ptr [rbp-0x2D10]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmulps   ymm14, ymm7, ymmword ptr [rbp-0x2D30]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmulps   ymm14, ymm6, ymmword ptr [rbp-0x2D50]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rcx+0x20], ymm14
       vmulps   ymm14, ymm4, ymmword ptr [rbp-0x2D10]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmulps   ymm14, ymm12, ymmword ptr [rbp-0x2D30]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmulps   ymm14, ymm10, ymmword ptr [rbp-0x2D50]
       vaddps   ymm14, ymm14, ymmword ptr [rbp-0x3310]
 
G_M000_IG47:                ;; offset=0x4463
       vmovups  ymmword ptr [rcx+0x40], ymm14
       vmovups  ymm14, ymmword ptr [rax]
       vmovups  ymmword ptr [rbp-0x3310], ymm14
       vmovups  ymm14, ymmword ptr [rdx]
       vcmpgeps ymm14, ymm14, ymmword ptr [rbp-0x550]
       vpand    ymm14, ymm14, ymmword ptr [rbp-0x3310]
       vmovups  ymmword ptr [rax], ymm14
       lea      rcx, bword ptr [rbx+0x120]
       lea      rdx, bword ptr [rbx+0x240]
       lea      rax, bword ptr [rbx+0x340]
       vmovups  ymm14, ymmword ptr [rbp-0x1650]
       vsubps   ymm1, ymm14, ymm1
       vmulps   ymm1, ymm1, ymm3
       vmovups  ymm3, ymmword ptr [rbp-0x1670]
       vsubps   ymm0, ymm3, ymm0
       vmulps   ymm0, ymm0, ymm11
       vaddps   ymm0, ymm0, ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x1690]
       vsubps   ymm5, ymm1, ymm5
       vmulps   ymm2, ymm5, ymm2
       vaddps   ymm0, ymm2, ymm0
       vmovups  ymm2, ymmword ptr [rbp-0x530]
       vmulps   ymm0, ymm2, ymm0
       vmovups  ymmword ptr [rdx], ymm0
       vmovups  ymm11, ymmword ptr [rbp-0x1470]
       vaddps   ymm0, ymm11, ymm14
       vmovups  ymm14, ymmword ptr [rbp-0x1490]
       vaddps   ymm2, ymm14, ymm3
       vmovups  ymm14, ymmword ptr [rbp-0x14B0]
       vaddps   ymm1, ymm14, ymm1
       vmulps   ymm3, ymm15, ymm0
       vmulps   ymm5, ymm8, ymm2
       vaddps   ymm3, ymm5, ymm3
       vmulps   ymm5, ymm13, ymm1
       vaddps   ymm3, ymm5, ymm3
       vmovups  ymmword ptr [rcx], ymm3
       vmulps   ymm3, ymm9, ymm0
       vmulps   ymm5, ymm7, ymm2
       vaddps   ymm3, ymm5, ymm3
       vmulps   ymm5, ymm6, ymm1
       vaddps   ymm3, ymm5, ymm3
       vmovups  ymmword ptr [rcx+0x20], ymm3
       vmulps   ymm0, ymm4, ymm0
       vmulps   ymm2, ymm12, ymm2
       vaddps   ymm0, ymm2, ymm0
       vmulps   ymm1, ymm10, ymm1
       vaddps   ymm0, ymm1, ymm0
       vmovups  ymmword ptr [rcx+0x40], ymm0
       vmovups  ymm0, ymmword ptr [rax]
       vmovups  ymm1, ymmword ptr [rdx]
       vcmpgeps ymm1, ymm1, ymmword ptr [rbp-0x550]
       vpand    ymm0, ymm1, ymm0
       vmovups  ymmword ptr [rax], ymm0
       lea      rcx, bword ptr [rbx+0x180]
       vmulps   ymm0, ymm15, ymmword ptr [rbp-0x2A0]
       vmulps   ymm1, ymm8, ymmword ptr [rbp-0x280]
       vaddps   ymm0, ymm1, ymm0
       vmulps   ymm1, ymm13, ymmword ptr [rbp-0x260]
       vaddps   ymm0, ymm1, ymm0
 
G_M000_IG48:                ;; offset=0x459A
       vmovups  ymmword ptr [rcx], ymm0
       vmulps   ymm0, ymm9, ymmword ptr [rbp-0x2A0]
       vmulps   ymm1, ymm7, ymmword ptr [rbp-0x280]
       vaddps   ymm0, ymm1, ymm0
       vmulps   ymm1, ymm6, ymmword ptr [rbp-0x260]
       vaddps   ymm0, ymm1, ymm0
       vmovups  ymmword ptr [rcx+0x20], ymm0
       vmulps   ymm0, ymm4, ymmword ptr [rbp-0x2A0]
       vmulps   ymm1, ymm12, ymmword ptr [rbp-0x280]
       vaddps   ymm0, ymm1, ymm0
       vmulps   ymm1, ymm10, ymmword ptr [rbp-0x260]
       vaddps   ymm0, ymm1, ymm0
       vmovups  ymmword ptr [rcx+0x40], ymm0
       vxorps   ymm0, ymm0, ymm0
       vmovups  ymmword ptr [rbx+0x260], ymm0
       vbroadcastss ymm0, dword ptr [reloc @RWD352]
       vmovups  ymmword ptr [rbx+0x280], ymm0
       vbroadcastss ymm0, dword ptr [reloc @RWD356]
       vmovups  ymmword ptr [rbx+0x2A0], ymm0
       vbroadcastss ymm0, dword ptr [reloc @RWD360]
       vmovups  ymmword ptr [rbx+0x2C0], ymm0
       jmp      SHORT G_M000_IG50
 
G_M000_IG49:                ;; offset=0x4629
       mov      rcx, rbx
       mov      edx, 864
       call     [CORINFO_HELP_MEMZERO]
 
G_M000_IG50:                ;; offset=0x4637
       nop      
 
G_M000_IG51:                ;; offset=0x4638
       vzeroupper 
       vmovaps  xmm6, xmmword ptr [rsp+0x3380]
       vmovaps  xmm7, xmmword ptr [rsp+0x3370]
       vmovaps  xmm8, xmmword ptr [rsp+0x3360]
       vmovaps  xmm9, xmmword ptr [rsp+0x3350]
       vmovaps  xmm10, xmmword ptr [rsp+0x3340]
       vmovaps  xmm11, xmmword ptr [rsp+0x3330]
       vmovaps  xmm12, xmmword ptr [rsp+0x3320]
       vmovaps  xmm13, xmmword ptr [rsp+0x3310]
       vmovaps  xmm14, xmmword ptr [rsp+0x3300]
       vmovaps  xmm15, xmmword ptr [rsp+0x32F0]
       add      rsp, 0x3390
       pop      rbx
       pop      rsi
       pop      rdi
       pop      r14
       pop      rbp
       ret      
 
RWD00  	dd	3F800000h		;         1
RWD04  	dd	80000000h		;        -0
RWD08  	dd	00000000h, 00000000h, 00000000h, 00000000h, 00000000h, 00000000h
RWD32  	dq	2EDBE6FF2EDBE6FFh, 2EDBE6FF2EDBE6FFh, 2EDBE6FF2EDBE6FFh, 2EDBE6FF2EDBE6FFh
RWD64  	dq	3F80000000000000h, 4040000040000000h, 40A0000040800000h, 40E0000040C00000h
RWD96  	dq	358637BD358637BDh, 358637BD358637BDh, 358637BD358637BDh, 358637BD358637BDh
RWD128 	dq	322BCC77322BCC77h, 322BCC77322BCC77h, 322BCC77322BCC77h, 322BCC77322BCC77h
RWD160 	dd	3F3504F3h		;  0.707107
RWD164 	dd	7FFFFFFFh		;       nan
RWD168 	dd	3F7FF972h		;    0.9999
RWD172 	dd	469C395Dh		;   19996.7
RWD176 	dd	00000000h, 00000000h, 00000000h, 00000000h
RWD192 	dq	283424DC283424DCh, 283424DC283424DCh, 283424DC283424DCh, 283424DC283424DCh
RWD224 	dq	2B8CBCCC2B8CBCCCh, 2B8CBCCC2B8CBCCCh, 2B8CBCCC2B8CBCCCh, 2B8CBCCC2B8CBCCCh
RWD256 	dq	3F0000003F000000h, 3F0000003F000000h, 3F0000003F000000h, 3F0000003F000000h
RWD288 	dd	26901D7Dh		;     1e-15
RWD292 	dd	3CB851ECh		;    0.0225
RWD296 	dd	00000000h, 00000000h, 00000000h, 00000000h, 00000000h, 00000000h
RWD320 	dq	4234FED74234FED7h, 4234FED74234FED7h, 4234FED74234FED7h, 4234FED74234FED7h
RWD352 	dd	00000001h		; 1.4013e-45
RWD356 	dd	00000002h		; 2.8026e-45
RWD360 	dd	00000003h		; 4.2039e-45

; Total bytes of code 18083

; Assembly listing for method BepuPhysics.CollisionDetection.DepthRefiner`6[BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder,BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder]:FindMinimumDepth(byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,byref,int) (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rbp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 39 single block inlinees; 0 inlinees without PGO data

G_M000_IG01:                ;; offset=0x0000
       push     rbp
       push     r15
       push     r14
       push     r13
       push     r12
       push     rdi
       push     rsi
       push     rbx
       sub      rsp, 776
       vmovaps  xmmword ptr [rsp+0x2F0], xmm6
       vmovaps  xmmword ptr [rsp+0x2E0], xmm7
       vmovaps  xmmword ptr [rsp+0x2D0], xmm8
       vmovaps  xmmword ptr [rsp+0x2C0], xmm9
       vmovaps  xmmword ptr [rsp+0x2B0], xmm10
       vmovaps  xmmword ptr [rsp+0x2A0], xmm11
       vmovaps  xmmword ptr [rsp+0x290], xmm12
       vmovaps  xmmword ptr [rsp+0x280], xmm13
       vmovaps  xmmword ptr [rsp+0x270], xmm14
       vmovaps  xmmword ptr [rsp+0x260], xmm15
       lea      rbp, [rsp+0x340]
       mov      r15, rcx
       mov      r13, rdx
       mov      r14, r8
       mov      rbx, r9
       mov      r12, bword ptr [rbp+0x40]
       mov      rdx, bword ptr [rbp+0x50]
       mov      r10, bword ptr [rbp+0x60]
       mov      rdi, bword ptr [rbp+0x70]
       mov      rsi, bword ptr [rbp+0x78]
 
G_M000_IG02:                ;; offset=0x0095
       mov      r8, bword ptr [rbp+0x68]
       vmovups  ymm6, ymmword ptr [r8]
       vmovups  ymm0, ymmword ptr [rdx]
       vcmpltps ymm0, ymm0, ymm6
       mov      r8, bword ptr [rbp+0x58]
       vpor     ymm0, ymm0, ymmword ptr [r8]
       vmovups  ymmword ptr [rbp-0x110], ymm0
       mov      r8, bword ptr [rbp+0x48]
       vmovdqu  ymm0, ymmword ptr [r8]
       vmovdqu  ymmword ptr [rsi], ymm0
       vmovdqu  ymm0, ymmword ptr [r8+0x20]
       vmovdqu  ymmword ptr [rsi+0x20], ymm0
       vmovdqu  ymm0, ymmword ptr [r8+0x40]
       vmovdqu  ymmword ptr [rsi+0x40], ymm0
       vmovups  ymm0, ymmword ptr [rdx]
       vmovups  ymmword ptr [rdi], ymm0
       vxorps   ymm0, ymm0, ymm0
       vpcmpgtd ymm0, ymm0, ymmword ptr [rbp-0x110]
       vpcmpeqd ymm1, ymm1, ymm1
       vptest   ymm0, ymm1
       jb       G_M000_IG08
 
G_M000_IG03:                ;; offset=0x00FE
       vxorps   ymm0, ymm0, ymm0
       vmovdqu  ymmword ptr [rbp-0x1D0], ymm0
       vmovdqu  ymmword ptr [rbp-0x1B0], ymm0
       vmovdqu  ymmword ptr [rbp-0x190], ymm0
       vxorps   ymm0, ymm0, ymm0
       vmovdqu  ymmword ptr [rbp-0x230], ymm0
       vmovdqu  ymmword ptr [rbp-0x210], ymm0
       vmovdqu  ymmword ptr [rbp-0x1F0], ymm0
       mov      bword ptr [rsp+0x20], rsi
       mov      bword ptr [rsp+0x28], rdi
       mov      bword ptr [rbp+0x60], r10
       mov      bword ptr [rsp+0x30], r10
       lea      rdx, [rbp-0x170]
       mov      qword ptr [rsp+0x38], rdx
       lea      rdx, [rbp-0x1D0]
       lea      r8, [rbp-0x230]
       lea      r9, [rbp-0x110]
       mov      rcx, r12
       vextractf128 xmm7, ymm6
       call     [BepuPhysics.CollisionDetection.DepthRefiner`6[BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder,BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder]:GetNextNormal[BepuPhysics.CollisionDetection.DepthRefiner`6+HasNoNewSupport[BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder,BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder]](byref,byref,byref,byref,byref,byref,byref,byref)]
       xor      eax, eax
       mov      dword ptr [rbp-0x234], eax
       cmp      eax, dword ptr [rbp+0x88]
       vinsertf128 ymm6, ymm6, xmm7
       jge      G_M000_IG07
 
G_M000_IG04:                ;; offset=0x0193
       vxorps   ymm0, ymm0, ymm0
       vpcmpgtd ymm0, ymm0, ymmword ptr [rbp-0x110]
       vpcmpeqd ymm1, ymm1, ymm1
       vptest   ymm0, ymm1
       jb       G_M000_IG07
       vmovups  ymm0, ymmword ptr [rbp-0x150]
       vxorps   ymm1, ymm1, ymm1
       vcmpgtps ymm0, ymm0, ymm1
       vmovups  ymm1, ymmword ptr [r15+0x20]
       vbroadcastss ymm2, dword ptr [reloc @RWD00]
       vxorps   ymm3, ymm2, ymm1
       vblendvps ymm0, ymm3, ymm1, ymm0
       vmovups  ymmword ptr [rbp-0x2D8], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x170]
       vmulps   ymm0, ymm0, ymmword ptr [rbp-0x170]
       vmovups  ymm1, ymmword ptr [rbp-0x130]
       vmulps   ymm1, ymm1, ymmword ptr [rbp-0x130]
       vaddps   ymm0, ymm1, ymm0
       vsqrtps  ymm0, ymm0
       vmovups  ymm1, ymmword ptr [r15]
       vdivps   ymm1, ymm1, ymm0
       vbroadcastss ymm3, dword ptr [reloc @RWD04]
       vcmpgtps ymm0, ymm0, ymm3
       vmulps   ymm4, ymm1, ymmword ptr [rbp-0x170]
       vandps   ymm4, ymm4, ymm0
       vxorps   ymm5, ymm5, ymm5
       vandnps  ymm5, ymm0, ymm5
       vorps    ymm4, ymm5, ymm4
       vmovups  ymmword ptr [rbp-0x2F8], ymm4
       vmulps   ymm1, ymm1, ymmword ptr [rbp-0x130]
       vandps   ymm1, ymm1, ymm0
       vxorps   ymm4, ymm4, ymm4
       vandnps  ymm0, ymm0, ymm4
       vorps    ymm0, ymm0, ymm1
       vmovups  ymmword ptr [rbp-0x2B8], ymm0
       vxorps   ymm0, ymm2, ymmword ptr [rbp-0x170]
       vxorps   ymm1, ymm2, ymmword ptr [rbp-0x150]
       vxorps   ymm4, ymm2, ymmword ptr [rbp-0x130]
       vmovups  ymm5, ymmword ptr [rbx]
       vmulps   ymm7, ymm5, ymm0
       vmovups  ymm8, ymmword ptr [rbx+0x20]
       vmulps   ymm9, ymm8, ymm1
       vaddps   ymm7, ymm9, ymm7
       vmovups  ymm9, ymmword ptr [rbx+0x40]
       vmulps   ymm10, ymm9, ymm4
       vaddps   ymm7, ymm10, ymm7
       vmovups  ymm10, ymmword ptr [rbx+0x60]
       vmulps   ymm11, ymm10, ymm0
       vmovups  ymm12, ymmword ptr [rbx+0x80]
       vmulps   ymm13, ymm12, ymm1
       vaddps   ymm11, ymm13, ymm11
       vmovups  ymm13, ymmword ptr [rbx+0xA0]
       vmulps   ymm14, ymm13, ymm4
       vaddps   ymm11, ymm14, ymm11
       vmovups  ymm14, ymmword ptr [rbx+0xC0]
       vmulps   ymm0, ymm14, ymm0
       vmovups  ymm15, ymmword ptr [rbx+0xE0]
       vmulps   ymm1, ymm15, ymm1
 
G_M000_IG05:                ;; offset=0x02DC
       vaddps   ymm0, ymm1, ymm0
       vmovups  ymm1, ymmword ptr [rbx+0x100]
       vmulps   ymm4, ymm1, ymm4
       vaddps   ymm0, ymm4, ymm0
       vxorps   ymm4, ymm4, ymm4
       vcmpgtps ymm4, ymm11, ymm4
       vmovups  ymm11, ymmword ptr [r13+0x20]
       vxorps   ymm2, ymm2, ymm11
       vblendvps ymm2, ymm2, ymm11, ymm4
       vmulps   ymm4, ymm7, ymm7
       vmulps   ymm11, ymm0, ymm0
       vaddps   ymm4, ymm11, ymm4
       vsqrtps  ymm4, ymm4
       vmovups  ymm11, ymmword ptr [r13]
       vdivps   ymm11, ymm11, ymm4
       vcmpgtps ymm3, ymm4, ymm3
       vmulps   ymm4, ymm7, ymm11
       vandps   ymm4, ymm4, ymm3
       vxorps   ymm7, ymm7, ymm7
       vandnps  ymm7, ymm3, ymm7
       vorps    ymm4, ymm7, ymm4
       vmulps   ymm0, ymm0, ymm11
       vandps   ymm0, ymm0, ymm3
       vxorps   ymm7, ymm7, ymm7
       vandnps  ymm3, ymm3, ymm7
       vorps    ymm0, ymm3, ymm0
       vmulps   ymm3, ymm5, ymm4
       vmulps   ymm5, ymm10, ymm2
       vaddps   ymm3, ymm5, ymm3
       vmulps   ymm5, ymm14, ymm0
       vaddps   ymm3, ymm5, ymm3
       vmulps   ymm5, ymm8, ymm4
       vmulps   ymm7, ymm12, ymm2
       vaddps   ymm5, ymm7, ymm5
       vmulps   ymm7, ymm15, ymm0
       vaddps   ymm5, ymm7, ymm5
       vmulps   ymm4, ymm9, ymm4
       vmulps   ymm2, ymm13, ymm2
       vaddps   ymm2, ymm2, ymm4
       vmulps   ymm0, ymm1, ymm0
       vaddps   ymm0, ymm0, ymm2
       vaddps   ymm3, ymm3, ymmword ptr [r14]
       vaddps   ymm5, ymm5, ymmword ptr [r14+0x20]
       vaddps   ymm0, ymm0, ymmword ptr [r14+0x40]
       vmovups  ymm1, ymmword ptr [rbp-0x2F8]
       vsubps   ymm1, ymm1, ymm3
       vmovups  ymmword ptr [rbp-0x298], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x2D8]
       vsubps   ymm1, ymm1, ymm5
       vmovups  ymmword ptr [rbp-0x278], ymm1
       vmovups  ymm1, ymmword ptr [rbp-0x2B8]
       vsubps   ymm0, ymm1, ymm0
       vmovups  ymmword ptr [rbp-0x258], ymm0
       vmovups  ymm0, ymmword ptr [rbp-0x298]
       vmulps   ymm0, ymm0, ymmword ptr [rbp-0x170]
       vmovups  ymm1, ymmword ptr [rbp-0x278]
       vmulps   ymm1, ymm1, ymmword ptr [rbp-0x150]
       vaddps   ymm0, ymm1, ymm0
       vmovups  ymm1, ymmword ptr [rbp-0x258]
 
G_M000_IG06:                ;; offset=0x0408
       vmulps   ymm1, ymm1, ymmword ptr [rbp-0x130]
       vaddps   ymm0, ymm1, ymm0
       vmovups  ymm1, ymmword ptr [rdi]
       vcmpltps ymm2, ymm0, ymm1
       vmovups  ymm3, ymmword ptr [rbp-0x110]
       vpandn   ymm2, ymm3, ymm2
       vandps   ymm0, ymm2, ymm0
       vandnps  ymm1, ymm2, ymm1
       vorps    ymm0, ymm1, ymm0
       vmovups  ymmword ptr [rdi], ymm0
       vandps   ymm0, ymm2, ymmword ptr [rbp-0x170]
       vandnps  ymm1, ymm2, ymmword ptr [rsi]
       vorps    ymm0, ymm1, ymm0
       vmovups  ymmword ptr [rsi], ymm0
       vandps   ymm0, ymm2, ymmword ptr [rbp-0x150]
       vandnps  ymm1, ymm2, ymmword ptr [rsi+0x20]
       vorps    ymm0, ymm1, ymm0
       vmovups  ymmword ptr [rsi+0x20], ymm0
       vandps   ymm0, ymm2, ymmword ptr [rbp-0x130]
       vandnps  ymm1, ymm2, ymmword ptr [rsi+0x40]
       vorps    ymm0, ymm1, ymm0
       vmovups  ymmword ptr [rsi+0x40], ymm0
       vmovups  ymm0, ymmword ptr [rdi]
       vcmpleps ymm0, ymm0, ymm6
       vpor     ymm0, ymm0, ymmword ptr [rbp-0x110]
       vmovups  ymmword ptr [rbp-0x110], ymm0
       vxorps   ymm0, ymm0, ymm0
       vpcmpgtd ymm0, ymm0, ymmword ptr [rbp-0x110]
       vpcmpeqd ymm1, ymm1, ymm1
       vptest   ymm0, ymm1
       jb       SHORT G_M000_IG07
       mov      bword ptr [rsp+0x20], rsi
       mov      bword ptr [rsp+0x28], rdi
       mov      r11, bword ptr [rbp+0x60]
       mov      bword ptr [rsp+0x30], r11
       lea      rdx, [rbp-0x170]
       mov      qword ptr [rsp+0x38], rdx
       lea      rdx, [rbp-0x298]
       lea      r8, [rbp-0x2F8]
       lea      r9, [rbp-0x110]
       mov      rcx, r12
       vextractf128 xmm7, ymm6
       call     [BepuPhysics.CollisionDetection.DepthRefiner`6[BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder,BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder]:GetNextNormal[BepuPhysics.CollisionDetection.DepthRefiner`6+HasNewSupport[BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder,BepuPhysics.Collidables.Cylinder,BepuPhysics.Collidables.CylinderWide,BepuPhysics.Collidables.CylinderSupportFinder]](byref,byref,byref,byref,byref,byref,byref,byref)]
       mov      eax, dword ptr [rbp-0x234]
       inc      eax
       mov      r10d, dword ptr [rbp+0x88]
       cmp      eax, r10d
       vinsertf128 ymm6, ymm6, xmm7
       mov      dword ptr [rbp-0x234], eax
       mov      dword ptr [rbp+0x88], r10d
       jl       G_M000_IG04
 
G_M000_IG07:                ;; offset=0x0517
       vbroadcastss ymm0, dword ptr [reloc @RWD08]
       vdivps   ymm0, ymm0, ymmword ptr [r12+0x300]
       lea      rax, bword ptr [r12+0x60]
       vmulps   ymm1, ymm0, ymmword ptr [r12+0xC0]
       vmulps   ymm2, ymm1, ymmword ptr [rax]
       vmulps   ymm3, ymm1, ymmword ptr [rax+0x20]
       vmulps   ymm4, ymm1, ymmword ptr [rax+0x40]
       lea      rax, bword ptr [r12+0x160]
       vmulps   ymm1, ymm0, ymmword ptr [r12+0x1C0]
       vmulps   ymm5, ymm1, ymmword ptr [rax]
       vmulps   ymm6, ymm1, ymmword ptr [rax+0x20]
       vmulps   ymm7, ymm1, ymmword ptr [rax+0x40]
       lea      rax, bword ptr [r12+0x260]
       vmulps   ymm1, ymm0, ymmword ptr [r12+0x2C0]
       vmulps   ymm0, ymm1, ymmword ptr [rax]
       vmulps   ymm8, ymm1, ymmword ptr [rax+0x20]
       vmulps   ymm1, ymm1, ymmword ptr [rax+0x40]
       vaddps   ymm2, ymm2, ymm5
       mov      rbx, bword ptr [rbp+0x80]
       vmovups  ymmword ptr [rbx], ymm2
       vaddps   ymm2, ymm3, ymm6
       vmovups  ymmword ptr [rbx+0x20], ymm2
       vaddps   ymm2, ymm4, ymm7
       vmovups  ymmword ptr [rbx+0x40], ymm2
       vaddps   ymm0, ymm0, ymmword ptr [rbx]
       vmovups  ymmword ptr [rbx], ymm0
       vaddps   ymm0, ymm8, ymmword ptr [rbx+0x20]
       vmovups  ymmword ptr [rbx+0x20], ymm0
       vaddps   ymm0, ymm1, ymmword ptr [rbx+0x40]
       vmovups  ymmword ptr [rbx+0x40], ymm0
       jmp      SHORT G_M000_IG09
 
G_M000_IG08:                ;; offset=0x05C6
       mov      rax, bword ptr [rbp+0x80]
       vxorps   ymm0, ymm0, ymm0
       vmovdqu  ymmword ptr [rax], ymm0
       vmovdqu  ymmword ptr [rax+0x20], ymm0
       vmovdqu  ymmword ptr [rax+0x40], ymm0
 
G_M000_IG09:                ;; offset=0x05DF
       vzeroupper 
       vmovaps  xmm6, xmmword ptr [rsp+0x2F0]
       vmovaps  xmm7, xmmword ptr [rsp+0x2E0]
       vmovaps  xmm8, xmmword ptr [rsp+0x2D0]
       vmovaps  xmm9, xmmword ptr [rsp+0x2C0]
       vmovaps  xmm10, xmmword ptr [rsp+0x2B0]
       vmovaps  xmm11, xmmword ptr [rsp+0x2A0]
       vmovaps  xmm12, xmmword ptr [rsp+0x290]
       vmovaps  xmm13, xmmword ptr [rsp+0x280]
       vmovaps  xmm14, xmmword ptr [rsp+0x270]
       vmovaps  xmm15, xmmword ptr [rsp+0x260]
       add      rsp, 776
       pop      rbx
       pop      rsi
       pop      rdi
       pop      r12
       pop      r13
       pop      r14
       pop      r15
       pop      rbp
       ret      
 
RWD00  	dd	80000000h		;        -0
RWD04  	dd	322BCC77h		;     1e-08
RWD08  	dd	3F800000h		;         1

; Total bytes of code 1616

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

; Assembly listing for method AosBaselines.CylinderPairScalarTester:Test(byref,byref,float,byref,byref,byref,byref) (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rbp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 261 single block inlinees; 49 inlinees without PGO data

G_M000_IG01:                ;; offset=0x0000
       push     rbp
       push     rdi
       push     rsi
       push     rbx
       sub      rsp, 0x5A8
       vmovaps  xmmword ptr [rsp+0x590], xmm6
       vmovaps  xmmword ptr [rsp+0x580], xmm7
       vmovaps  xmmword ptr [rsp+0x570], xmm8
       vmovaps  xmmword ptr [rsp+0x560], xmm9
       vmovaps  xmmword ptr [rsp+0x550], xmm10
       vmovaps  xmmword ptr [rsp+0x540], xmm11
       vmovaps  xmmword ptr [rsp+0x530], xmm12
       vmovaps  xmmword ptr [rsp+0x520], xmm13
       vmovaps  xmmword ptr [rsp+0x510], xmm14
       vmovaps  xmmword ptr [rsp+0x500], xmm15
       lea      rbp, [rsp+0x5C0]
       vxorps   xmm4, xmm4, xmm4
       vmovdqu  ymmword ptr [rbp-0x140], ymm4
       vmovdqu  ymmword ptr [rbp-0x120], ymm4
       vmovdqu  ymmword ptr [rbp-0x100], ymm4
       vmovdqu  ymmword ptr [rbp-0xE0], ymm4
       mov      rsi, rcx
       mov      rdi, rdx
       mov      rbx, bword ptr [rbp+0x40]
 
G_M000_IG02:                ;; offset=0x009B
       vxorps   ymm0, ymm0, ymm0
       vmovdqu  ymmword ptr [rbx], ymm0
       vmovdqu  ymmword ptr [rbx+0x20], ymm0
       vmovdqu  ymmword ptr [rbx+0x40], ymm0
       mov      r8, bword ptr [rbp+0x30]
       vmovups  xmm0, xmmword ptr [r8]
       vmovaps  xmm1, xmm0
       vmovaps  xmm3, xmm0
       vaddss   xmm1, xmm1, xmm3
       vmovshdup xmm3, xmm0
       vaddss   xmm4, xmm3, xmm3
       vunpckhps xmm5, xmm0, xmm0
       vaddss   xmm6, xmm5, xmm5
       vmovaps  xmm7, xmm0
       vmulss   xmm7, xmm1, xmm7
       vmulss   xmm8, xmm4, xmm3
       vmulss   xmm9, xmm6, xmm5
       vmulss   xmm3, xmm1, xmm3
       vmulss   xmm10, xmm1, xmm5
       vshufps  xmm0, xmm0, xmm0, -1
       vmulss   xmm1, xmm1, xmm0
       vmulss   xmm5, xmm4, xmm5
       vmulss   xmm4, xmm4, xmm0
       vmulss   xmm0, xmm6, xmm0
       vmovss   xmm6, dword ptr [reloc @RWD00]
       vsubss   xmm11, xmm6, xmm8
       vsubss   xmm11, xmm11, xmm9
       vaddss   xmm12, xmm3, xmm0
       vinsertps xmm11, xmm11, xmm12, 16
       vsubss   xmm12, xmm10, xmm4
       vinsertps xmm11, xmm11, xmm12, 40
       vsubss   xmm0, xmm3, xmm0
       vsubss   xmm3, xmm6, xmm7
       vsubss   xmm7, xmm3, xmm9
       vinsertps xmm0, xmm0, xmm7, 16
       vaddss   xmm7, xmm5, xmm1
       vinsertps xmm0, xmm0, xmm7, 40
       vaddss   xmm4, xmm10, xmm4
       vsubss   xmm1, xmm5, xmm1
       vinsertps xmm1, xmm4, xmm1, 16
       vsubss   xmm3, xmm3, xmm8
       vinsertps xmm1, xmm1, xmm3, 40
       mov      r8, bword ptr [rbp+0x38]
       vmovups  xmm3, xmmword ptr [r8]
       vmovaps  xmm4, xmm3
       vmovaps  xmm5, xmm3
       vaddss   xmm4, xmm4, xmm5
       vmovshdup xmm5, xmm3
       vaddss   xmm7, xmm5, xmm5
       vunpckhps xmm8, xmm3, xmm3
       vaddss   xmm9, xmm8, xmm8
       vmovaps  xmm10, xmm3
       vmulss   xmm10, xmm4, xmm10
       vmulss   xmm12, xmm7, xmm5
       vmulss   xmm13, xmm9, xmm8
       vmulss   xmm5, xmm4, xmm5
       vmulss   xmm14, xmm4, xmm8
       vshufps  xmm3, xmm3, xmm3, -1
       vmulss   xmm4, xmm4, xmm3
 
G_M000_IG03:                ;; offset=0x01A5
       vmulss   xmm8, xmm7, xmm8
       vmulss   xmm7, xmm7, xmm3
       vmulss   xmm3, xmm9, xmm3
       vsubss   xmm9, xmm6, xmm12
       vsubss   xmm9, xmm9, xmm13
       vaddss   xmm15, xmm5, xmm3
       vinsertps xmm9, xmm9, xmm15, 16
       vsubss   xmm15, xmm14, xmm7
       vinsertps xmm9, xmm9, xmm15, 40
       vsubss   xmm3, xmm5, xmm3
       vsubss   xmm5, xmm6, xmm10
       vsubss   xmm10, xmm5, xmm13
       vinsertps xmm3, xmm3, xmm10, 16
       vaddss   xmm10, xmm8, xmm4
       vinsertps xmm10, xmm3, xmm10, 40
       vaddss   xmm3, xmm14, xmm7
       vsubss   xmm4, xmm8, xmm4
       vinsertps xmm3, xmm3, xmm4, 16
       vsubss   xmm4, xmm5, xmm12
       vinsertps xmm7, xmm3, xmm4, 40
       vmovshdup xmm3, xmm9
       vunpckhps xmm4, xmm9, xmm9
       vunpckhps xmm5, xmm10, xmm10
       vmovaps  xmm8, xmm9
       vmovaps  xmm12, xmm10
       vinsertps xmm8, xmm8, xmm12, 16
       vmovaps  xmm12, xmm7
       vinsertps xmm8, xmm8, xmm12, 40
       vmovshdup xmm12, xmm10
       vinsertps xmm3, xmm3, xmm12, 16
       vmovshdup xmm12, xmm7
       vinsertps xmm3, xmm3, xmm12, 40
       vinsertps xmm4, xmm4, xmm5, 16
       vunpckhps xmm5, xmm7, xmm7
       vinsertps xmm4, xmm4, xmm5, 40
       vmovaps  xmm5, xmm8
       vmovaps  xmm12, xmm11
       vbroadcastss xmm12, xmm12
       vmulps   xmm5, xmm12, xmm5
       vmovaps  xmm12, xmm3
       vmovshdup xmm13, xmm11
       vbroadcastss xmm13, xmm13
       vmulps   xmm12, xmm13, xmm12
       vaddps   xmm5, xmm12, xmm5
       vmovaps  xmm12, xmm4
       vunpckhps xmm11, xmm11, xmm11
       vbroadcastss xmm11, xmm11
       vmulps   xmm11, xmm11, xmm12
       vaddps   xmm5, xmm11, xmm5
       vmovsd   qword ptr [rbp-0xE8], xmm5
       vextractps dword ptr [rbp-0xE0], xmm5, 2
       vmovaps  xmm5, xmm8
       vmovaps  xmm11, xmm0
       vbroadcastss xmm11, xmm11
       vmulps   xmm5, xmm11, xmm5
       vmovaps  xmm11, xmm3
       vmovshdup xmm12, xmm0
       vbroadcastss xmm12, xmm12
       vmulps   xmm11, xmm12, xmm11
       vaddps   xmm5, xmm11, xmm5
       vmovaps  xmm11, xmm4
       vunpckhps xmm0, xmm0, xmm0
 
G_M000_IG04:                ;; offset=0x02D8
       vbroadcastss xmm0, xmm0
       vmulps   xmm0, xmm0, xmm11
       vaddps   xmm0, xmm0, xmm5
       vmovsd   qword ptr [rbp-0xDC], xmm0
       vextractps dword ptr [rbp-0xD4], xmm0, 2
       vmovaps  xmm0, xmm1
       vbroadcastss xmm0, xmm0
       vmulps   xmm0, xmm0, xmm8
       vmovshdup xmm5, xmm1
       vbroadcastss xmm5, xmm5
       vmulps   xmm3, xmm5, xmm3
       vaddps   xmm0, xmm3, xmm0
       vunpckhps xmm1, xmm1, xmm1
       vbroadcastss xmm1, xmm1
       vmulps   xmm1, xmm1, xmm4
       vaddps   xmm0, xmm1, xmm0
       vmovsd   qword ptr [rbp-0xD0], xmm0
       vextractps dword ptr [rbp-0xC8], xmm0, 2
       vmovsd   xmm0, qword ptr [r9]
       vinsertps xmm0, xmm0, dword ptr [r9+0x08], 40
       vinsertps xmm0, xmm0, xmm0, 56
       vmovaps  xmm1, xmm9
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm1, xmm1, xmm0
       vpermilps xmm3, xmm1, -11
       vaddps   xmm3, xmm3, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm3
       vmovaps  xmm3, xmm10
       vinsertps xmm3, xmm3, xmm3, 56
       vmulps   xmm3, xmm3, xmm0
       vpermilps xmm4, xmm3, -11
       vaddps   xmm4, xmm4, xmm3
       vpermilps xmm3, xmm3, -86
       vaddps   xmm3, xmm3, xmm4
       vmovaps  xmm4, xmm7
       vinsertps xmm4, xmm4, xmm4, 56
       vmulps   xmm0, xmm4, xmm0
       vinsertps xmm1, xmm1, xmm3, 16
       vpermilps xmm3, xmm0, -11
       vaddps   xmm3, xmm3, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm3
       vinsertps xmm8, xmm1, xmm0, 40
       vmovaps  xmmword ptr [rbp-0x100], xmm8
       vmovaps  xmm0, xmm8
       vxorps   xmm0, xmm0, xmmword ptr [reloc @RWD16]
       vmovaps  xmmword ptr [rbp-0x110], xmm0
       vmovaps  xmm0, xmmword ptr [rbp-0x110]
       vinsertps xmm0, xmm0, xmm0, 56
       vmulps   xmm0, xmm0, xmm0
       vpermilps xmm1, xmm0, -11
       vaddps   xmm1, xmm1, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm1
       vsqrtss  xmm0, xmm0, xmm0
       vdivss   xmm1, xmm6, xmm0
       vbroadcastss xmm1, xmm1
       vmovaps  xmm3, xmmword ptr [rbp-0x110]
       vmulps   xmm1, xmm1, xmm3
 
G_M000_IG05:                ;; offset=0x041C
       vmovaps  xmmword ptr [rbp-0x120], xmm1
       vmovss   xmm1, dword ptr [reloc @RWD32]
       vucomiss xmm1, xmm0
       jbe      SHORT G_M000_IG07
 
G_M000_IG06:                ;; offset=0x0432
       vmovaps  xmm0, xmmword ptr [rbp-0x120]
       vxorps   xmm1, xmm1, xmm1
       vinsertps xmm0, xmm0, xmm1, 1
       vmovaps  xmmword ptr [rbp-0x120], xmm0
       vmovaps  xmm0, xmmword ptr [rbp-0x120]
       vmovaps  xmm1, xmm6
       vinsertps xmm0, xmm0, xmm1, 16
       vmovaps  xmmword ptr [rbp-0x120], xmm0
       vmovaps  xmm0, xmmword ptr [rbp-0x120]
       vxorps   xmm1, xmm1, xmm1
       vinsertps xmm0, xmm0, xmm1, 36
       vmovaps  xmmword ptr [rbp-0x120], xmm0
 
G_M000_IG07:                ;; offset=0x0480
       vmovss   dword ptr [rbp+0x20], xmm2
       vxorps   xmm11, xmm2, xmmword ptr [reloc @RWD16]
       vmovss   xmm0, dword ptr [rsi+0x04]
       vmaxss   xmm0, xmm0, dword ptr [rsi]
       vmovss   xmm1, dword ptr [rdi+0x04]
       vmaxss   xmm1, xmm1, dword ptr [rdi]
       vminss   xmm0, xmm0, xmm1
       lea      r8, [rbp-0x120]
       mov      qword ptr [rsp+0x20], r8
       vmulss   xmm0, xmm0, dword ptr [reloc @RWD36]
       vmovss   dword ptr [rsp+0x28], xmm0
       vmovss   dword ptr [rsp+0x30], xmm11
       lea      r8, [rbp-0x124]
       mov      qword ptr [rsp+0x38], r8
       lea      r8, [rbp-0x120]
       mov      qword ptr [rsp+0x40], r8
       lea      r8, [rbp-0x140]
       mov      qword ptr [rsp+0x48], r8
       mov      dword ptr [rsp+0x50], 25
       lea      r8, [rbp-0x110]
       lea      r9, [rbp-0xE8]
       mov      rcx, rdi
       mov      rdx, rsi
       call     [AosBaselines.ScalarDepthRefiner`4[BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar,BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar]:FindMinimumDepth(byref,byref,byref,byref,byref,float,float,byref,byref,byref,int)]
       vucomiss xmm11, dword ptr [rbp-0x124]
       ja       G_M000_IG32
 
G_M000_IG08:                ;; offset=0x0517
       vmovups  xmm0, xmmword ptr [rbp-0xDC]
       vinsertps xmm1, xmm0, xmm0, 56
       vmovaps  xmmword ptr [rbp-0x4C0], xmm1
       vmovaps  xmm2, xmmword ptr [rbp-0x120]
       vmovaps  xmmword ptr [rbp-0x4E0], xmm2
       vinsertps xmm3, xmm2, xmm2, 56
       vmulps   xmm3, xmm3, xmm1
       vpermilps xmm4, xmm3, -11
       vaddps   xmm4, xmm4, xmm3
       vpermilps xmm3, xmm3, -86
       vaddps   xmm3, xmm3, xmm4
       vdivss   xmm4, xmm6, xmm3
       vmovss   dword ptr [rbp-0x144], xmm4
       vmovss   xmm5, dword ptr [rbp-0x11C]
       vdivss   xmm11, xmm6, xmm5
       vmovss   dword ptr [rbp-0x148], xmm11
       vbroadcastss xmm12, xmm3
       vxorps   xmm13, xmm13, xmm13
       vcmpgtps xmm12, xmm12, xmm13
       vmovss   xmm13, dword ptr [rsi+0x04]
       vxorps   xmm14, xmm13, xmmword ptr [reloc @RWD16]
       vandps   xmm14, xmm14, xmm12
       vandnps  xmm13, xmm12, xmm13
       vorps    xmm13, xmm13, xmm14
       vbroadcastss xmm13, xmm13
       vmulps   xmm13, xmm13, xmm0
       vmovaps  xmm14, xmmword ptr [rbp-0x110]
       vmovaps  xmmword ptr [rbp-0x4F0], xmm14
       vaddps   xmm13, xmm13, xmm14
       vbroadcastss xmm15, xmm5
       vmovaps  xmmword ptr [rbp-0x4D0], xmm15
       vxorps   xmm1, xmm1, xmm1
       vcmpltps xmm1, xmm15, xmm1
       vmovss   xmm15, dword ptr [rdi+0x04]
       vxorps   xmm14, xmm15, xmmword ptr [reloc @RWD16]
       vblendvps xmm1, xmm15, xmm14, xmm1
       vandps   xmm3, xmm3, xmmword ptr [reloc @RWD48]
       vmovss   dword ptr [rbp-0x4AC], xmm3
       vucomiss xmm3, dword ptr [reloc @RWD64]
       seta     al
       movzx    rax, al
       vandps   xmm5, xmm5, xmmword ptr [reloc @RWD48]
       vmovss   dword ptr [rbp-0x4A8], xmm5
       vucomiss xmm5, dword ptr [reloc @RWD64]
       seta     cl
       movzx    rcx, cl
       vxorps   xmm14, xmm14, xmm14
       vxorps   xmm15, xmm15, xmm15
       vxorps   xmm5, xmm5, xmm5
       vmovaps  xmmword ptr [rbp-0x160], xmm5
       vxorps   xmm3, xmm3, xmm3
       vmovaps  xmmword ptr [rbp-0x170], xmm3
       vmovaps  xmm2, xmmword ptr [rbp-0x140]
       vmovss   xmm3, dword ptr [rbp-0x124]
       vxorps   xmm3, xmm3, xmmword ptr [reloc @RWD16]
       vbroadcastss xmm3, xmm3
       vmulps   xmm3, xmm3, xmmword ptr [rbp-0x4E0]
       vaddps   xmm2, xmm3, xmm2
       vsubps   xmm2, xmm2, xmmword ptr [rbp-0x4F0]
       vinsertps xmm3, xmm2, xmm2, 56
 
G_M000_IG09:                ;; offset=0x0683
       vmulps   xmm3, xmm3, xmmword ptr [rbp-0x4C0]
       vpermilps xmm5, xmm3, -11
       vaddps   xmm5, xmm5, xmm3
       vpermilps xmm3, xmm3, -86
       vaddps   xmm3, xmm3, xmm5
       vbroadcastss xmm3, xmm3
       vmulps   xmm3, xmm3, xmm0
       vsubps   xmm2, xmm2, xmm3
       vmovaps  xmmword ptr [rbp-0x180], xmm2
       vmovss   xmm3, dword ptr [rbp-0x140]
       vinsertps xmm3, xmm3, dword ptr [rbp-0x138], 28
       vxorps   xmm5, xmm0, xmmword ptr [reloc @RWD16]
       vandps   xmm5, xmm5, xmm12
       vandnps  xmm0, xmm12, xmm0
       vorps    xmm0, xmm0, xmm5
       vmovaps  xmmword ptr [rbp-0x190], xmm0
       vmovaps  xmm5, xmm0
       vmovaps  xmm12, xmm13
       test     eax, ecx
       jne      SHORT G_M000_IG11
 
G_M000_IG10:                ;; offset=0x06F0
       jmp      G_M000_IG22
 
G_M000_IG11:                ;; offset=0x06F5
       vmovss   xmm14, dword ptr [rbp-0x4AC]
       vmovss   dword ptr [rbp-0x1B4], xmm14
       vmovss   xmm0, dword ptr [rbp-0x4A8]
       vmovss   dword ptr [rbp-0x1B8], xmm0
       vmovss   xmm14, dword ptr [reloc @RWD68]
       vmovss   dword ptr [rbp-0x528], xmm14
       vucomiss xmm14, xmm0
       seta     dl
       movzx    rdx, dl
       vmovaps  xmm0, xmm3
       vucomiss xmm14, dword ptr [rbp-0x1B4]
       seta     r8b
       movzx    r8, r8b
       test     r8d, edx
       jne      G_M000_IG21
       vmovaps  xmm0, xmmword ptr [rbp-0x120]
       vmovshdup xmm15, xmm12
       vsubss   xmm15, xmm15, xmm1
       vmulss   xmm15, xmm15, xmm11
       vmovss   dword ptr [rbp-0x2E4], xmm15
       vmovaps  xmm15, xmm0
       vmulss   xmm15, xmm15, dword ptr [rbp-0x2E4]
       vmovaps  xmm11, xmm12
       vsubss   xmm11, xmm11, xmm15
       vunpckhps xmm0, xmm0, xmm0
       vmulss   xmm0, xmm0, dword ptr [rbp-0x2E4]
       vunpckhps xmm15, xmm12, xmm12
       vsubss   xmm0, xmm15, xmm0
       vinsertps xmm0, xmm11, xmm0, 28
       vmovaps  xmm11, xmm0
       vmovaps  xmm15, xmm0
       vmulss   xmm11, xmm11, xmm15
       vmovshdup xmm15, xmm0
       vmovshdup xmm2, xmm0
       vmulss   xmm2, xmm15, xmm2
       vaddss   xmm2, xmm11, xmm2
       vsqrtss  xmm2, xmm2, xmm2
       vdivss   xmm11, xmm6, xmm2
       vmovss   dword ptr [rbp-0x1BC], xmm11
       vbroadcastss xmm15, xmm11
       vmulps   xmm0, xmm15, xmm0
       vbroadcastss xmm15, xmm2
       vcmpltps xmm15, xmm15, xmmword ptr [reloc @RWD80]
       vandps   xmm11, xmm15, xmmword ptr [reloc @RWD96]
       vandnps  xmm0, xmm15, xmm0
       vorps    xmm0, xmm0, xmm11
       vmovaps  xmm11, xmm0
       vmovss   xmm15, dword ptr [rdi]
       vmovss   dword ptr [rbp-0x52C], xmm15
       vbroadcastss xmm4, xmm15
       vmulps   xmm4, xmm4, xmm11
       vmovaps  xmm11, xmm4
       vxorps   xmm11, xmm11, xmmword ptr [reloc @RWD16]
       vmovsd   qword ptr [rbp-0x1C8], xmm11
       vmovups  xmmword ptr [rbp-0x550], xmm11
       vmovaps  xmm11, xmm4
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vmovups  xmm11, xmmword ptr [rbp-0x550]
       vsubps   xmm11, xmm11, xmmword ptr [rbp-0x560]
       vmovsd   qword ptr [rbp-0x2C8], xmm11
       vmulss   xmm14, xmm15, xmm15
       vmovss   dword ptr [rbp-0x53C], xmm14
       vmovss   xmm14, dword ptr [rsi]
       vmovss   dword ptr [rbp-0x530], xmm14
 
G_M000_IG12:                ;; offset=0x0863
       vmulss   xmm14, xmm14, xmm14
       vmovss   dword ptr [rbp-0x4A4], xmm14
       vmovss   xmm14, dword ptr [rbp-0x53C]
       vsubss   xmm14, xmm14, dword ptr [rbp-0x4A4]
       vmulss   xmm14, xmm14, dword ptr [rbp-0x1BC]
       vaddss   xmm14, xmm14, xmm2
       vmulss   xmm14, xmm14, dword ptr [reloc @RWD112]
       vmovss   dword ptr [rbp-0x1CC], xmm14
       vxorps   xmm14, xmm14, xmm14
       vmaxss   xmm14, xmm14, dword ptr [rbp-0x1CC]
       vminss   xmm2, xmm2, xmm14
       vbroadcastss xmm2, xmm2
       vmovaps  xmm14, xmm0
       vmulps   xmm2, xmm2, xmm14
       vmovshdup xmm14, xmm0
       vmovups  xmmword ptr [rbp-0x560], xmm14
       vmovaps  xmm14, xmm0
       vxorps   xmm14, xmm14, xmmword ptr [reloc @RWD16]
       vmovups  xmm15, xmmword ptr [rbp-0x560]
       vinsertps xmm14, xmm15, xmm14, 28
       vmovaps  xmm15, xmm2
       vmovups  xmmword ptr [rbp-0x560], xmm15
       vmovaps  xmm15, xmm14
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x560]
       vmovsd   qword ptr [rbp-0x1D8], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x120]
       vmovaps  xmm11, xmm4
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vmovss   xmm11, dword ptr [rbp-0x1C8]
       vmovss   dword ptr [rbp-0x53C], xmm11
       vmovups  xmm11, xmmword ptr [rbp-0x560]
       vinsertps xmm11, xmm11, dword ptr [rbp-0x53C], 16
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vmovaps  xmm11, xmm2
       vmovss   dword ptr [rbp-0x53C], xmm11
       vmovups  xmm11, xmmword ptr [rbp-0x560]
       vinsertps xmm11, xmm11, dword ptr [rbp-0x53C], 32
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vmovss   xmm11, dword ptr [rbp-0x1D8]
       vmovss   dword ptr [rbp-0x53C], xmm11
       vmovups  xmm11, xmmword ptr [rbp-0x560]
       vinsertps xmm11, xmm11, dword ptr [rbp-0x53C], 48
       vmovaps  xmmword ptr [rbp-0x300], xmm11
       vbroadcastss xmm11, xmm1
       vmovaps  xmmword ptr [rbp-0x310], xmm11
       vmovshdup xmm11, xmm4
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vmovsd   xmm11, qword ptr [rbp-0x1C8]
       vmovshdup xmm11, xmm11
       vmovss   dword ptr [rbp-0x53C], xmm11
       vmovups  xmm11, xmmword ptr [rbp-0x560]
       vinsertps xmm11, xmm11, dword ptr [rbp-0x53C], 16
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vmovshdup xmm11, xmm2
       vmovss   dword ptr [rbp-0x53C], xmm11
       vmovups  xmm11, xmmword ptr [rbp-0x560]
       vinsertps xmm11, xmm11, dword ptr [rbp-0x53C], 32
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vmovsd   xmm11, qword ptr [rbp-0x1D8]
       vmovshdup xmm11, xmm11
       vmovss   dword ptr [rbp-0x53C], xmm11
 
G_M000_IG13:                ;; offset=0x0A14
       vmovups  xmm11, xmmword ptr [rbp-0x560]
       vinsertps xmm11, xmm11, dword ptr [rbp-0x53C], 48
       vmovaps  xmmword ptr [rbp-0x320], xmm11
       vmovaps  xmm11, xmm12
       vbroadcastss xmm11, xmm11
       vsubps   xmm11, xmm11, xmmword ptr [rbp-0x300]
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vbroadcastss xmm11, dword ptr [rbp-0xDC]
       vmulps   xmm11, xmm11, xmmword ptr [rbp-0x560]
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vmovshdup xmm11, xmm12
       vbroadcastss xmm11, xmm11
       vmovaps  xmmword ptr [rbp-0x500], xmm11
       vsubps   xmm11, xmm11, xmmword ptr [rbp-0x310]
       vmovups  xmmword ptr [rbp-0x550], xmm11
       vbroadcastss xmm11, dword ptr [rbp-0xD8]
       vmulps   xmm11, xmm11, xmmword ptr [rbp-0x550]
       vaddps   xmm11, xmm11, xmmword ptr [rbp-0x560]
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vunpckhps xmm11, xmm12, xmm12
       vbroadcastss xmm11, xmm11
       vmovaps  xmmword ptr [rbp-0x510], xmm11
       vsubps   xmm11, xmm11, xmmword ptr [rbp-0x320]
       vmovups  xmmword ptr [rbp-0x550], xmm11
       vbroadcastss xmm11, dword ptr [rbp-0xD4]
       vmulps   xmm11, xmm11, xmmword ptr [rbp-0x550]
       vaddps   xmm11, xmm11, xmmword ptr [rbp-0x560]
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vbroadcastss xmm11, dword ptr [rbp-0x144]
       vmulps   xmm11, xmm11, xmmword ptr [rbp-0x560]
       vmovaps  xmmword ptr [rbp-0x330], xmm11
       vmovaps  xmm11, xmm15
       vbroadcastss xmm11, xmm11
       vmulps   xmm11, xmm11, xmmword ptr [rbp-0x330]
       vaddps   xmm11, xmm11, xmmword ptr [rbp-0x300]
       vmovaps  xmmword ptr [rbp-0x340], xmm11
       vmovaps  xmm11, xmmword ptr [rbp-0x330]
       vmulps   xmm11, xmm11, xmmword ptr [rbp-0x4D0]
       vaddps   xmm11, xmm11, xmmword ptr [rbp-0x310]
       vmovaps  xmmword ptr [rbp-0x350], xmm11
       vmovaps  xmm11, xmm12
       vbroadcastss xmm11, xmm11
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vmovaps  xmm11, xmmword ptr [rbp-0x340]
       vsubps   xmm11, xmm11, xmmword ptr [rbp-0x560]
       vmovaps  xmmword ptr [rbp-0x360], xmm11
       vmovaps  xmm11, xmmword ptr [rbp-0x350]
       vsubps   xmm11, xmm11, xmmword ptr [rbp-0x500]
       vmovaps  xmmword ptr [rbp-0x370], xmm11
       vunpckhps xmm15, xmm15, xmm15
       vbroadcastss xmm15, xmm15
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x330]
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x320]
       vsubps   xmm15, xmm15, xmmword ptr [rbp-0x510]
       vmovaps  xmmword ptr [rbp-0x380], xmm15
       vbroadcastss xmm15, dword ptr [rbp-0xE8]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x360]
       vbroadcastss xmm11, dword ptr [rbp-0xE4]
       vmulps   xmm11, xmm11, xmmword ptr [rbp-0x370]
       vaddps   xmm11, xmm11, xmm15
       vbroadcastss xmm15, dword ptr [rbp-0xE0]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x380]
 
G_M000_IG14:                ;; offset=0x0BE6
       vaddps   xmm11, xmm15, xmm11
       vmovaps  xmmword ptr [rbp-0x390], xmm11
       vbroadcastss xmm15, dword ptr [rbp-0xD0]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x360]
       vmovups  xmmword ptr [rbp-0x560], xmm15
       vbroadcastss xmm15, dword ptr [rbp-0xCC]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x370]
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x560]
       vmovups  xmmword ptr [rbp-0x560], xmm15
       vbroadcastss xmm15, dword ptr [rbp-0xC8]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x380]
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x560]
       vmovaps  xmmword ptr [rbp-0x3A0], xmm15
       vmovups  xmmword ptr [rbp-0x560], xmm11
       vmovaps  xmm11, xmm15
       vmovss   dword ptr [rbp-0x53C], xmm11
       vmovups  xmm11, xmmword ptr [rbp-0x560]
       vinsertps xmm11, xmm11, dword ptr [rbp-0x53C], 28
       vmovsd   qword ptr [rbp-0x1E0], xmm11
       vmovaps  xmm15, xmmword ptr [rbp-0x390]
       vmovshdup xmm15, xmm15
       vmovups  xmmword ptr [rbp-0x560], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x3A0]
       vmovshdup xmm15, xmm15
       vmovss   dword ptr [rbp-0x53C], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x560]
       vinsertps xmm15, xmm15, dword ptr [rbp-0x53C], 28
       vmovsd   qword ptr [rbp-0x1E8], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x390]
       vunpckhps xmm15, xmm15, xmm15
       vmovups  xmmword ptr [rbp-0x560], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x3A0]
       vunpckhps xmm15, xmm15, xmm15
       vmovss   dword ptr [rbp-0x53C], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x560]
       vinsertps xmm15, xmm15, dword ptr [rbp-0x53C], 28
       vmovsd   qword ptr [rbp-0x1F0], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x390]
       vshufps  xmm15, xmm15, xmm15, -1
       vmovups  xmmword ptr [rbp-0x560], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x3A0]
       vshufps  xmm15, xmm15, xmm15, -1
       vmovss   dword ptr [rbp-0x53C], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x560]
       vinsertps xmm15, xmm15, dword ptr [rbp-0x53C], 28
       vmovsd   qword ptr [rbp-0x1F8], xmm15
       vmovsd   xmm15, qword ptr [rbp-0x1E8]
       vmovups  xmmword ptr [rbp-0x560], xmm15
       vmovaps  xmm15, xmm11
       vmovups  xmmword ptr [rbp-0x550], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x560]
       vsubps   xmm15, xmm15, xmmword ptr [rbp-0x550]
       vmovsd   qword ptr [rbp-0x200], xmm15
       vmovss   dword ptr [rbp-0x53C], xmm15
       vmovss   xmm15, dword ptr [rbp-0x200]
       vmulss   xmm15, xmm15, dword ptr [rbp-0x53C]
       vmovss   dword ptr [rbp-0x53C], xmm15
       vmovsd   xmm15, qword ptr [rbp-0x200]
       vmovshdup xmm15, xmm15
 
G_M000_IG15:                ;; offset=0x0DAD
       vmovss   dword ptr [rbp-0x534], xmm15
       vmulss   xmm15, xmm15, xmm15
       vaddss   xmm15, xmm15, dword ptr [rbp-0x53C]
       vmovss   dword ptr [rbp-0x3A4], xmm15
       vdivss   xmm15, xmm6, xmm15
       vmovss   dword ptr [rbp-0x3A8], xmm15
       vmovaps  xmm15, xmm11
       vmovss   dword ptr [rbp-0x53C], xmm15
       vmovss   xmm15, dword ptr [rbp-0x200]
       vmulss   xmm15, xmm15, dword ptr [rbp-0x53C]
       vmovss   dword ptr [rbp-0x53C], xmm15
       vmovshdup xmm15, xmm11
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovss   xmm11, dword ptr [rbp-0x534]
       vmulss   xmm11, xmm15, xmm11
       vaddss   xmm11, xmm11, dword ptr [rbp-0x53C]
       vmovss   dword ptr [rbp-0x3AC], xmm11
       vmovss   xmm11, dword ptr [rbp-0x1E0]
       vmovss   xmm15, dword ptr [rbp-0x1E0]
       vmulss   xmm11, xmm11, xmm15
       vmovss   xmm15, dword ptr [rbp-0x538]
       vmulss   xmm15, xmm15, xmm15
       vaddss   xmm11, xmm11, xmm15
       vsubss   xmm11, xmm11, dword ptr [rbp-0x4A4]
       vmovss   dword ptr [rbp-0x3B0], xmm11
       vmovss   xmm15, dword ptr [rbp-0x3AC]
       vmulss   xmm15, xmm15, xmm15
       vmovss   xmm11, dword ptr [rbp-0x3A4]
       vmulss   xmm11, xmm11, dword ptr [rbp-0x3B0]
       vsubss   xmm11, xmm15, xmm11
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm11, xmm15, xmm11
       vsqrtss  xmm11, xmm11, xmm11
       vmulss   xmm11, xmm11, dword ptr [rbp-0x3A8]
       vmovss   dword ptr [rbp-0x3B4], xmm11
       vmovss   xmm11, dword ptr [rbp-0x3AC]
       vxorps   xmm11, xmm11, xmmword ptr [reloc @RWD16]
       vmulss   xmm11, xmm11, dword ptr [rbp-0x3A8]
       vmovss   dword ptr [rbp-0x3B8], xmm11
       vsubss   xmm15, xmm11, dword ptr [rbp-0x3B4]
       vmovss   dword ptr [rbp-0x204], xmm15
       vaddss   xmm15, xmm11, dword ptr [rbp-0x3B4]
       vmovss   dword ptr [rbp-0x208], xmm15
       vmovss   xmm11, dword ptr [rbp-0x3A4]
       vandps   xmm11, xmm11, xmmword ptr [reloc @RWD48]
       vbroadcastss xmm11, xmm11
       vcmpltps xmm11, xmm11, xmmword ptr [reloc @RWD128]
       vmovss   xmm15, dword ptr [rbp-0x3B8]
       vmovaps  xmmword ptr [rbp-0x520], xmm15
       vandps   xmm15, xmm15, xmm11
       vmovups  xmmword ptr [rbp-0x550], xmm15
       vmovss   xmm15, dword ptr [rbp-0x204]
       vandnps  xmm15, xmm11, xmm15
       vorps    xmm15, xmm15, xmmword ptr [rbp-0x550]
       vmovss   dword ptr [rbp-0x204], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x520]
       vandps   xmm15, xmm15, xmm11
       vmovups  xmmword ptr [rbp-0x550], xmm15
       vmovss   xmm15, dword ptr [rbp-0x208]
       vandnps  xmm11, xmm11, xmm15
 
G_M000_IG16:                ;; offset=0x0F58
       vorps    xmm15, xmm11, xmmword ptr [rbp-0x550]
       vmovss   dword ptr [rbp-0x208], xmm15
       vmovss   xmm11, dword ptr [rbp-0x204]
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm11, xmm11, xmm15
       vmovss   dword ptr [rbp-0x20C], xmm11
       vmovss   xmm15, dword ptr [rbp-0x208]
       vminss   xmm15, xmm15, dword ptr [reloc @RWD00]
       vmovss   dword ptr [rbp-0x210], xmm15
       vmovsd   xmm15, qword ptr [rbp-0x2C8]
       vbroadcastss xmm11, xmm11
       vmulps   xmm11, xmm11, xmm15
       vmovaps  xmm15, xmm4
       vaddps   xmm11, xmm11, xmm15
       vmovsd   xmm15, qword ptr [rbp-0x2C8]
       vmovups  xmmword ptr [rbp-0x550], xmm15
       vbroadcastss xmm15, dword ptr [rbp-0x210]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x550]
       vaddps   xmm4, xmm15, xmm4
       vmovsd   qword ptr [rbp-0x218], xmm4
       vmovsd   xmm15, qword ptr [rbp-0x1F8]
       vmovsd   xmm4, qword ptr [rbp-0x1F0]
       vsubps   xmm4, xmm15, xmm4
       vmovsd   qword ptr [rbp-0x220], xmm4
       vmovss   xmm15, dword ptr [rbp-0x530]
       vmovss   dword ptr [rbp-0x3F4], xmm15
       vmovss   xmm4, dword ptr [rbp-0x52C]
       vmovss   dword ptr [rbp-0x3F8], xmm4
       vmovsd   xmm4, qword ptr [rbp-0x1F0]
       vmovshdup xmm4, xmm4
       vmovshdup xmm15, xmm2
       vinsertps xmm4, xmm4, xmm15, 28
       vmovaps  xmmword ptr [rbp-0x3D0], xmm4
       vmovss   xmm15, dword ptr [rbp-0x220]
       vmovaps  xmm4, xmm14
       vinsertps xmm4, xmm15, xmm4, 28
       vmovaps  xmmword ptr [rbp-0x3E0], xmm4
       vmovsd   xmm15, qword ptr [rbp-0x220]
       vmovshdup xmm15, xmm15
       vmovshdup xmm4, xmm14
       vinsertps xmm4, xmm15, xmm4, 28
       vmovaps  xmmword ptr [rbp-0x3F0], xmm4
       vmovss   xmm15, dword ptr [rbp-0x3F4]
       vinsertps xmm15, xmm15, dword ptr [rbp-0x3F8], 28
       vmovaps  xmmword ptr [rbp-0x410], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x3E0]
       vmulps   xmm15, xmm15, xmm15
       vmulps   xmm4, xmm4, xmm4
       vaddps   xmm4, xmm4, xmm15
       vmovaps  xmmword ptr [rbp-0x420], xmm4
       vbroadcastss xmm15, dword ptr [reloc @RWD00]
       vdivps   xmm15, xmm15, xmm4
       vmovaps  xmmword ptr [rbp-0x430], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1F0]
       vmovaps  xmm4, xmm2
       vinsertps xmm4, xmm15, xmm4, 28
       vmulps   xmm15, xmm4, xmmword ptr [rbp-0x3E0]
       vmovups  xmmword ptr [rbp-0x550], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x3D0]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x3F0]
 
G_M000_IG17:                ;; offset=0x10F6
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x550]
       vmovaps  xmmword ptr [rbp-0x440], xmm15
       vmulps   xmm4, xmm4, xmm4
       vmovaps  xmm15, xmmword ptr [rbp-0x3D0]
       vmulps   xmm15, xmm15, xmm15
       vaddps   xmm4, xmm15, xmm4
       vmovaps  xmm15, xmmword ptr [rbp-0x410]
       vmulps   xmm15, xmm15, xmm15
       vsubps   xmm4, xmm4, xmm15
       vmulps   xmm4, xmm4, xmmword ptr [rbp-0x420]
       vmovaps  xmm15, xmmword ptr [rbp-0x440]
       vmulps   xmm15, xmm15, xmm15
       vsubps   xmm4, xmm15, xmm4
       vxorps   xmm15, xmm15, xmm15
       vmaxps   xmm4, xmm15, xmm4
       vsqrtps  xmm4, xmm4
       vmulps   xmm4, xmm4, xmmword ptr [rbp-0x430]
       vmovaps  xmmword ptr [rbp-0x450], xmm4
       vmovaps  xmm15, xmmword ptr [rbp-0x440]
       vxorps   xmm15, xmm15, xmmword ptr [reloc @RWD16]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x430]
       vmovaps  xmmword ptr [rbp-0x460], xmm15
       vsubps   xmm4, xmm15, xmm4
       vmovaps  xmmword ptr [rbp-0x470], xmm4
       vaddps   xmm4, xmm15, xmmword ptr [rbp-0x450]
       vmovaps  xmmword ptr [rbp-0x480], xmm4
       vmovaps  xmm4, xmmword ptr [rbp-0x420]
       vandps   xmm4, xmm4, xmmword ptr [reloc @RWD48]
       vcmpltps xmm4, xmm4, xmmword ptr [reloc @RWD128]
       vandps   xmm15, xmm4, xmm15
       vmovups  xmmword ptr [rbp-0x550], xmm15
       vandnps  xmm15, xmm4, xmmword ptr [rbp-0x470]
       vorps    xmm15, xmm15, xmmword ptr [rbp-0x550]
       vmovaps  xmmword ptr [rbp-0x470], xmm15
       vandps   xmm15, xmm4, xmmword ptr [rbp-0x460]
       vandnps  xmm4, xmm4, xmmword ptr [rbp-0x480]
       vorps    xmm4, xmm4, xmm15
       vmovss   xmm15, dword ptr [rbp-0x470]
       vmovss   dword ptr [rbp-0x224], xmm15
       vmovaps  xmm15, xmm4
       vmovss   dword ptr [rbp-0x228], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x470]
       vmovshdup xmm15, xmm15
       vmovshdup xmm4, xmm4
       vmovss   dword ptr [rbp-0x22C], xmm4
       vmovss   xmm4, dword ptr [rbp-0x224]
       vmaxss   xmm4, xmm4, xmm15
       vmovss   dword ptr [rbp-0x230], xmm4
       vmovss   xmm15, dword ptr [rbp-0x228]
       vminss   xmm15, xmm15, dword ptr [rbp-0x22C]
       vmovss   dword ptr [rbp-0x234], xmm15
       vmovaps  xmm15, xmm14
       vbroadcastss xmm4, xmm4
       vmulps   xmm4, xmm4, xmm15
       vmovaps  xmm15, xmm14
       vmovups  xmmword ptr [rbp-0x550], xmm15
       vbroadcastss xmm15, dword ptr [rbp-0x234]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x550]
       vmovsd   qword ptr [rbp-0x248], xmm15
       vmovaps  xmm15, xmm2
       vaddps   xmm4, xmm4, xmm15
 
G_M000_IG18:                ;; offset=0x1292
       vmovsd   qword ptr [rbp-0x240], xmm4
       vmovaps  xmm15, xmm2
       vmovups  xmmword ptr [rbp-0x550], xmm15
       vmovsd   xmm15, qword ptr [rbp-0x248]
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x550]
       vmovsd   qword ptr [rbp-0x248], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1B4]
       vsubss   xmm15, xmm15, dword ptr [rbp-0x528]
       vmulss   xmm15, xmm15, dword ptr [reloc @RWD144]
       vmovss   dword ptr [rbp-0x53C], xmm15
       vbroadcastss xmm15, dword ptr [reloc @RWD00]
       vminss   xmm15, xmm15, dword ptr [rbp-0x53C]
       vmovss   dword ptr [rbp-0x53C], xmm15
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm15, xmm15, dword ptr [rbp-0x53C]
       vmovss   dword ptr [rbp-0x53C], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1B8]
       vsubss   xmm15, xmm15, dword ptr [rbp-0x528]
       vmulss   xmm15, xmm15, dword ptr [reloc @RWD144]
       vmovss   dword ptr [rbp-0x540], xmm15
       vbroadcastss xmm15, dword ptr [reloc @RWD00]
       vminss   xmm15, xmm15, dword ptr [rbp-0x540]
       vmovss   dword ptr [rbp-0x540], xmm15
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm15, xmm15, dword ptr [rbp-0x540]
       vmulss   xmm15, xmm15, dword ptr [rbp-0x53C]
       vsubss   xmm4, xmm6, xmm15
       vmovss   dword ptr [rbp-0x24C], xmm4
       vmovaps  xmm4, xmm3
       vsubps   xmm2, xmm4, xmm2
       vmovsd   qword ptr [rbp-0x258], xmm2
       vmovaps  xmm4, xmm14
       vmovss   dword ptr [rbp-0x524], xmm2
       vmulss   xmm4, xmm4, xmm2
       vmovshdup xmm14, xmm14
       vmovsd   xmm2, qword ptr [rbp-0x258]
       vmovshdup xmm2, xmm2
       vmulss   xmm14, xmm14, xmm2
       vaddss   xmm4, xmm4, xmm14
       vmovss   dword ptr [rbp-0x25C], xmm4
       vmovaps  xmm14, xmm0
       vmulss   xmm14, xmm14, dword ptr [rbp-0x524]
       vmovshdup xmm0, xmm0
       vmulss   xmm0, xmm0, xmm2
       vaddss   xmm0, xmm14, xmm0
       vandps   xmm2, xmm0, xmmword ptr [reloc @RWD48]
       vbroadcastss xmm2, xmm2
       vandps   xmm14, xmm4, xmmword ptr [reloc @RWD48]
       vbroadcastss xmm14, xmm14
       vcmpgtps xmm2, xmm2, xmm14
       vbroadcastss xmm14, xmm0
       vxorps   xmm4, xmm4, xmm4
       vcmpgtps xmm4, xmm14, xmm4
       vandps   xmm4, xmm4, xmm2
       vmovaps  xmmword ptr [rbp-0x270], xmm4
       vbroadcastss xmm0, xmm0
       vxorps   xmm14, xmm14, xmm14
       vcmpleps xmm0, xmm0, xmm14
       vmovaps  xmmword ptr [rbp-0x2E0], xmm0
       vbroadcastss xmm14, dword ptr [rbp-0x25C]
       vxorps   xmm4, xmm4, xmm4
       vcmpltps xmm4, xmm14, xmm4
 
G_M000_IG19:                ;; offset=0x1426
       vandnps  xmm4, xmm2, xmm4
       vmovaps  xmmword ptr [rbp-0x280], xmm4
       vbroadcastss xmm14, dword ptr [rbp-0x25C]
       vxorps   xmm4, xmm4, xmm4
       vcmpgeps xmm4, xmm14, xmm4
       vandnps  xmm4, xmm2, xmm4
       vmovaps  xmmword ptr [rbp-0x290], xmm4
       vmovaps  xmm14, xmm3
       vbroadcastss xmm4, dword ptr [rbp-0x24C]
       vmulps   xmm4, xmm4, xmm14
       vmovaps  xmm14, xmm11
       vbroadcastss xmm0, xmm15
       vmulps   xmm0, xmm0, xmm14
       vandps   xmm2, xmm2, xmmword ptr [rbp-0x2E0]
       vmovaps  xmm14, xmm4
       vaddps   xmm0, xmm0, xmm14
       vmovaps  xmm14, xmmword ptr [rbp-0x270]
       vandps   xmm0, xmm0, xmm14
       vandnps  xmm11, xmm14, xmm11
       vorps    xmm11, xmm11, xmm0
       vmovsd   xmm0, qword ptr [rbp-0x218]
       vmovaps  xmm14, xmm0
       vbroadcastss xmm0, xmm15
       vmulps   xmm0, xmm0, xmm14
       vmovaps  xmm14, xmm4
       vaddps   xmm0, xmm0, xmm14
       vandps   xmm0, xmm0, xmm2
       vmovsd   xmm14, qword ptr [rbp-0x218]
       vandnps  xmm2, xmm2, xmm14
       vorps    xmm0, xmm2, xmm0
       vmovsd   qword ptr [rbp-0x488], xmm0
       vmovsd   xmm2, qword ptr [rbp-0x240]
       vmovaps  xmm14, xmm2
       vbroadcastss xmm0, xmm15
       vmulps   xmm0, xmm0, xmm14
       vmovaps  xmm14, xmm4
       vaddps   xmm0, xmm0, xmm14
       vmovaps  xmm14, xmmword ptr [rbp-0x280]
       vandps   xmm0, xmm0, xmm14
       vandnps  xmm2, xmm14, xmm2
       vorps    xmm0, xmm2, xmm0
       vmovsd   xmm2, qword ptr [rbp-0x248]
       vmovaps  xmm14, xmm2
       vbroadcastss xmm15, xmm15
       vmulps   xmm14, xmm15, xmm14
       vaddps   xmm4, xmm14, xmm4
       vmovaps  xmm14, xmmword ptr [rbp-0x290]
       vandps   xmm4, xmm4, xmm14
       vandnps  xmm2, xmm14, xmm2
       vorps    xmm2, xmm2, xmm4
       vmovsd   xmm4, qword ptr [rbp-0x488]
       vmovaps  xmm14, xmm4
       vinsertps xmm14, xmm14, xmm1, 16
       vmovshdup xmm4, xmm4
       vinsertps xmm15, xmm14, xmm4, 40
       vmovaps  xmm4, xmm0
       vinsertps xmm4, xmm4, xmm1, 16
       vmovshdup xmm0, xmm0
       vinsertps xmm0, xmm4, xmm0, 40
       vmovaps  xmm4, xmm2
       vinsertps xmm4, xmm4, xmm1, 16
       vmovshdup xmm2, xmm2
       vinsertps xmm2, xmm4, xmm2, 40
 
G_M000_IG20:                ;; offset=0x157B
       vmovss   xmm4, dword ptr [rbp-0x210]
       vucomiss xmm4, dword ptr [rbp-0x20C]
       seta     dl
       mov      byte  ptr [rbx+0x5D], dl
       movzx    rdx, byte  ptr [rbx+0x5D]
       mov      byte  ptr [rbx+0x5E], dl
       movzx    rdx, byte  ptr [rbx+0x5D]
       vmovss   xmm4, dword ptr [rbp-0x234]
       vucomiss xmm4, dword ptr [rbp-0x230]
       seta     r8b
       movzx    r8, r8b
       and      edx, r8d
       mov      byte  ptr [rbx+0x5F], dl
       vmovsd   qword ptr [rbp-0x160], xmm0
       vextractps dword ptr [rbp-0x158], xmm0, 2
       vmovsd   qword ptr [rbp-0x170], xmm2
       vextractps dword ptr [rbp-0x168], xmm2, 2
       vmovaps  xmm0, xmm11
       vmovss   xmm11, dword ptr [rbp-0x148]
 
G_M000_IG21:                ;; offset=0x15EB
       vmovaps  xmm14, xmm0
       vinsertps xmm14, xmm14, xmm1, 16
       vmovshdup xmm0, xmm0
       vinsertps xmm14, xmm14, xmm0, 40
       mov      byte  ptr [rbx+0x5C], 1
 
G_M000_IG22:                ;; offset=0x1603
       vmovups  xmm0, xmmword ptr [rbp-0xE8]
       vinsertps xmm0, xmm0, xmm0, 56
       vmovaps  xmm11, xmmword ptr [rbp-0x120]
       vinsertps xmm11, xmm11, xmm11, 56
       vmulps   xmm0, xmm11, xmm0
       vpermilps xmm11, xmm0, -11
       vaddps   xmm11, xmm11, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm11
       vmovaps  xmm11, xmmword ptr [rbp-0xD0]
       vinsertps xmm11, xmm11, xmm11, 56
       vmovaps  xmm4, xmmword ptr [rbp-0x120]
       vinsertps xmm4, xmm4, xmm4, 56
       vmulps   xmm4, xmm4, xmm11
       vpermilps xmm11, xmm4, -11
       vaddps   xmm11, xmm11, xmm4
       vpermilps xmm4, xmm4, -86
       vaddps   xmm4, xmm4, xmm11
       vmulss   xmm11, xmm0, xmm0
       vmulss   xmm2, xmm4, xmm4
       vaddss   xmm2, xmm11, xmm2
       vsqrtss  xmm2, xmm2, xmm2
       vdivss   xmm2, xmm6, xmm2
       vmulss   xmm0, xmm0, xmm2
       vbroadcastss xmm0, xmm0
       vmovups  xmm11, xmmword ptr [rbp-0xE8]
       vmulps   xmm0, xmm0, xmm11
       vmulss   xmm2, xmm4, xmm2
       vbroadcastss xmm2, xmm2
       vmovaps  xmm4, xmmword ptr [rbp-0xD0]
       vmulps   xmm2, xmm2, xmm4
       vaddps   xmm0, xmm2, xmm0
       vmovaps  xmmword ptr [rbp-0x1A0], xmm0
       vmovaps  xmm2, xmmword ptr [rbp-0x180]
       vmovaps  xmm4, xmmword ptr [rbp-0x110]
       vaddps   xmm2, xmm4, xmm2
       vmovaps  xmmword ptr [rbp-0x1B0], xmm2
       vmovaps  xmm4, xmm3
       vmovshdup xmm11, xmm3
       vinsertps xmm4, xmm4, xmm11, 42
       cmp      eax, ecx
       je       G_M000_IG27
       vmovaps  xmm5, xmm2
       vmovups  xmm14, xmmword ptr [rbp-0xDC]
       vaddps   xmm5, xmm14, xmm5
       vmovaps  xmm15, xmm4
       vinsertps xmm12, xmm15, xmm6, 16
       vunpckhps xmm11, xmm4, xmm4
       vinsertps xmm11, xmm12, xmm11, 40
       vmovaps  xmm12, xmmword ptr [rbp-0x120]
       vmovaps  xmm14, xmm13
       vmovaps  xmm15, xmm4
       vsubps   xmm14, xmm14, xmm15
       vinsertps xmm14, xmm14, xmm14, 56
       vmovups  xmm15, xmmword ptr [rbp-0xDC]
       vinsertps xmm15, xmm15, xmm15, 56
       vmulps   xmm14, xmm15, xmm14
       vpermilps xmm15, xmm14, -11
 
G_M000_IG23:                ;; offset=0x1744
       vaddps   xmm15, xmm15, xmm14
       vpermilps xmm14, xmm14, -86
       vaddps   xmm14, xmm14, xmm15
       vmulss   xmm14, xmm14, dword ptr [rbp-0x144]
       vbroadcastss xmm14, xmm14
       vmulps   xmm12, xmm14, xmm12
       vmovaps  xmm14, xmm4
       vaddps   xmm12, xmm12, xmm14
       vmovaps  xmm14, xmm13
       vsubps   xmm12, xmm12, xmm14
       vmovaps  xmm14, xmm12
       vinsertps xmm14, xmm14, xmm14, 56
       vmovups  xmm15, xmmword ptr [rbp-0xE8]
       vinsertps xmm15, xmm15, xmm15, 56
       vmulps   xmm14, xmm15, xmm14
       vpermilps xmm15, xmm14, -11
       vaddps   xmm15, xmm15, xmm14
       vpermilps xmm14, xmm14, -86
       vaddps   xmm14, xmm14, xmm15
       vinsertps xmm12, xmm12, xmm12, 56
       vmovaps  xmm15, xmmword ptr [rbp-0xD0]
       vinsertps xmm15, xmm15, xmm15, 56
       vmulps   xmm12, xmm15, xmm12
       vpermilps xmm15, xmm12, -11
       vaddps   xmm15, xmm15, xmm12
       vpermilps xmm12, xmm12, -86
       vaddps   xmm12, xmm12, xmm15
       vinsertps xmm12, xmm14, xmm12, 28
       vmovsd   qword ptr [rbp-0x298], xmm12
       vmovaps  xmm14, xmmword ptr [rbp-0x120]
       vmovshdup xmm15, xmm2
       vsubss   xmm15, xmm15, xmm1
       vmulss   xmm15, xmm15, dword ptr [rbp-0x148]
       vmovaps  xmm12, xmm14
       vmulss   xmm12, xmm12, xmm15
       vmovaps  xmm0, xmm2
       vsubss   xmm0, xmm0, xmm12
       vunpckhps xmm12, xmm14, xmm14
       vmulss   xmm12, xmm12, xmm15
       vunpckhps xmm14, xmm2, xmm2
       vsubss   xmm12, xmm14, xmm12
       vinsertps xmm0, xmm0, xmm12, 28
       vmovaps  xmm12, xmmword ptr [rbp-0x120]
       vmovaps  xmm14, xmm13
       vmovaps  xmm15, xmm11
       vsubps   xmm14, xmm14, xmm15
       vinsertps xmm14, xmm14, xmm14, 56
       vmovups  xmm15, xmmword ptr [rbp-0xDC]
       vinsertps xmm15, xmm15, xmm15, 56
       vmulps   xmm14, xmm15, xmm14
       vpermilps xmm15, xmm14, -11
       vaddps   xmm15, xmm15, xmm14
       vpermilps xmm14, xmm14, -86
       vaddps   xmm14, xmm14, xmm15
       vmulss   xmm14, xmm14, dword ptr [rbp-0x144]
       vbroadcastss xmm14, xmm14
       vmulps   xmm12, xmm14, xmm12
       vaddps   xmm11, xmm12, xmm11
       vmovaps  xmm12, xmm13
 
G_M000_IG24:                ;; offset=0x1890
       vsubps   xmm11, xmm11, xmm12
       vmovaps  xmm12, xmm11
       vinsertps xmm12, xmm12, xmm12, 56
       vmovups  xmm14, xmmword ptr [rbp-0xE8]
       vinsertps xmm14, xmm14, xmm14, 56
       vmulps   xmm12, xmm14, xmm12
       vpermilps xmm14, xmm12, -11
       vaddps   xmm14, xmm14, xmm12
       vpermilps xmm12, xmm12, -86
       vaddps   xmm12, xmm12, xmm14
       vinsertps xmm11, xmm11, xmm11, 56
       vmovaps  xmm14, xmmword ptr [rbp-0xD0]
       vinsertps xmm14, xmm14, xmm14, 56
       vmulps   xmm11, xmm14, xmm11
       vpermilps xmm14, xmm11, -11
       vaddps   xmm14, xmm14, xmm11
       vpermilps xmm11, xmm11, -86
       vaddps   xmm11, xmm11, xmm14
       vinsertps xmm11, xmm12, xmm11, 28
       vmovsd   qword ptr [rbp-0x2A0], xmm11
       vmovaps  xmm12, xmmword ptr [rbp-0x120]
       vmovshdup xmm14, xmm5
       vsubss   xmm14, xmm14, xmm1
       vmulss   xmm14, xmm14, dword ptr [rbp-0x148]
       vmovaps  xmm15, xmm12
       vmulss   xmm15, xmm15, xmm14
       vmovaps  xmm11, xmm5
       vsubss   xmm11, xmm11, xmm15
       vunpckhps xmm12, xmm12, xmm12
       vmulss   xmm12, xmm12, xmm14
       vunpckhps xmm5, xmm5, xmm5
       vsubss   xmm5, xmm5, xmm12
       vinsertps xmm5, xmm11, xmm5, 28
       mov      edx, -1
       xor      r8d, r8d
       test     eax, eax
       cmove    edx, r8d
       vmovd    xmm11, edx
       vpbroadcastd xmm11, xmm11
       vmovsd   xmm12, qword ptr [rbp-0x298]
       vandps   xmm12, xmm12, xmm11
       vandnps  xmm0, xmm11, xmm0
       vorps    xmm0, xmm0, xmm12
       vmovsd   xmm12, qword ptr [rbp-0x2A0]
       vandps   xmm12, xmm12, xmm11
       vandnps  xmm5, xmm11, xmm5
       vorps    xmm5, xmm5, xmm12
       vmovss   xmm12, dword ptr [rsi]
       vandps   xmm12, xmm12, xmm11
       vmovss   xmm14, dword ptr [rdi]
       vandnps  xmm14, xmm11, xmm14
       vorps    xmm12, xmm14, xmm12
       vmovss   dword ptr [rbp-0x2A4], xmm12
       vmovss   xmm14, dword ptr [rdi+0x04]
       vandps   xmm14, xmm14, xmm11
       vmovss   xmm15, dword ptr [rsi+0x04]
       vandnps  xmm15, xmm11, xmm15
       vorps    xmm14, xmm15, xmm14
       vmovaps  xmm15, xmm0
       vsubps   xmm5, xmm5, xmm15
       vmovaps  xmm15, xmm5
 
G_M000_IG25:                ;; offset=0x19D2
       vmovaps  xmm12, xmm5
       vmulss   xmm12, xmm15, xmm12
       vmovss   dword ptr [rbp-0x53C], xmm12
       vmovshdup xmm15, xmm5
       vmovshdup xmm12, xmm5
       vmulss   xmm12, xmm15, xmm12
       vaddss   xmm12, xmm12, dword ptr [rbp-0x53C]
       vmovss   dword ptr [rbp-0x48C], xmm12
       vdivss   xmm15, xmm6, xmm12
       vmovss   dword ptr [rbp-0x490], xmm15
       vmovaps  xmm15, xmm0
       vmovaps  xmm12, xmm5
       vmulss   xmm12, xmm15, xmm12
       vmovss   dword ptr [rbp-0x53C], xmm12
       vmovshdup xmm15, xmm0
       vmovshdup xmm12, xmm5
       vmulss   xmm12, xmm15, xmm12
       vaddss   xmm12, xmm12, dword ptr [rbp-0x53C]
       vmovss   dword ptr [rbp-0x494], xmm12
       vmovaps  xmm15, xmm0
       vmovaps  xmm12, xmm0
       vmulss   xmm12, xmm15, xmm12
       vmovss   dword ptr [rbp-0x53C], xmm12
       vmovshdup xmm15, xmm0
       vmovshdup xmm12, xmm0
       vmulss   xmm12, xmm15, xmm12
       vaddss   xmm12, xmm12, dword ptr [rbp-0x53C]
       vmovss   xmm15, dword ptr [rbp-0x2A4]
       vmulss   xmm15, xmm15, xmm15
       vsubss   xmm12, xmm12, xmm15
       vmovss   dword ptr [rbp-0x498], xmm12
       vmovss   xmm15, dword ptr [rbp-0x494]
       vmulss   xmm15, xmm15, xmm15
       vmovss   xmm12, dword ptr [rbp-0x48C]
       vmulss   xmm12, xmm12, dword ptr [rbp-0x498]
       vsubss   xmm12, xmm15, xmm12
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm12, xmm15, xmm12
       vsqrtss  xmm12, xmm12, xmm12
       vmulss   xmm12, xmm12, dword ptr [rbp-0x490]
       vmovss   dword ptr [rbp-0x49C], xmm12
       vmovss   xmm12, dword ptr [rbp-0x494]
       vxorps   xmm12, xmm12, xmmword ptr [reloc @RWD16]
       vmulss   xmm12, xmm12, dword ptr [rbp-0x490]
       vmovss   dword ptr [rbp-0x4A0], xmm12
       vsubss   xmm15, xmm12, dword ptr [rbp-0x49C]
       vaddss   xmm12, xmm12, dword ptr [rbp-0x49C]
       vmovss   xmm2, dword ptr [rbp-0x48C]
       vandps   xmm2, xmm2, xmmword ptr [reloc @RWD48]
       vbroadcastss xmm2, xmm2
       vcmpltps xmm2, xmm2, xmmword ptr [reloc @RWD128]
       vmovss   xmm8, dword ptr [rbp-0x4A0]
       vandps   xmm8, xmm8, xmm2
       vandnps  xmm15, xmm2, xmm15
       vorps    xmm15, xmm15, xmm8
       vmovss   xmm8, dword ptr [rbp-0x4A0]
       vandps   xmm8, xmm8, xmm2
       vandnps  xmm2, xmm2, xmm12
       vorps    xmm12, xmm2, xmm8
       vxorps   xmm2, xmm14, xmmword ptr [reloc @RWD16]
       vmaxss   xmm2, xmm2, xmm15
       vmovaps  xmm15, xmm14
       vminss   xmm15, xmm15, xmm2
 
G_M000_IG26:                ;; offset=0x1B54
       vminss   xmm12, xmm14, xmm12
       vmovaps  xmm2, xmm4
       vinsertps xmm2, xmm2, xmm15, 16
       vunpckhps xmm8, xmm4, xmm4
       vinsertps xmm2, xmm2, xmm8, 40
       vmovaps  xmmword ptr [rbp-0x2C0], xmm2
       vmovaps  xmm8, xmm5
       vmulss   xmm8, xmm15, xmm8
       vmovaps  xmm14, xmm0
       vaddss   xmm8, xmm8, xmm14
       vinsertps xmm8, xmm8, xmm1, 16
       vmovshdup xmm14, xmm5
       vmulss   xmm14, xmm15, xmm14
       vmovshdup xmm2, xmm0
       vaddss   xmm2, xmm14, xmm2
       vinsertps xmm2, xmm8, xmm2, 40
       vmovaps  xmm8, xmmword ptr [rbp-0x2C0]
       vandps   xmm8, xmm8, xmm11
       vandnps  xmm2, xmm11, xmm2
       vorps    xmm14, xmm2, xmm8
       vmovaps  xmm2, xmm4
       vinsertps xmm2, xmm2, xmm12, 16
       vunpckhps xmm4, xmm4, xmm4
       vinsertps xmm2, xmm2, xmm4, 40
       vmovaps  xmm4, xmm5
       vmulss   xmm4, xmm12, xmm4
       vmovaps  xmm8, xmm0
       vaddss   xmm4, xmm4, xmm8
       vinsertps xmm1, xmm4, xmm1, 16
       vmovshdup xmm4, xmm5
       vmulss   xmm4, xmm12, xmm4
       vmovshdup xmm0, xmm0
       vaddss   xmm0, xmm4, xmm0
       vinsertps xmm0, xmm1, xmm0, 40
       vandps   xmm1, xmm2, xmm11
       vandnps  xmm0, xmm11, xmm0
       vorps    xmm0, xmm0, xmm1
       mov      byte  ptr [rbx+0x5C], 1
       vucomiss xmm12, xmm15
       seta     dl
       mov      byte  ptr [rbx+0x5D], dl
       vmovaps  xmm1, xmmword ptr [rbp-0x190]
       vandps   xmm1, xmm1, xmm11
       vmovaps  xmm5, xmmword ptr [rbp-0x1A0]
       vmovaps  xmm2, xmm5
       vandnps  xmm2, xmm11, xmm2
       vorps    xmm1, xmm2, xmm1
       vmovaps  xmm2, xmm13
       vandps   xmm2, xmm2, xmm11
       vmovaps  xmm12, xmmword ptr [rbp-0x1B0]
       vmovaps  xmm4, xmm12
       vandnps  xmm4, xmm11, xmm4
       vorps    xmm2, xmm4, xmm2
       vmovaps  xmm5, xmm1
       vmovaps  xmm15, xmm0
       vmovaps  xmm4, xmm2
       vmovaps  xmm2, xmm12
       vmovaps  xmm12, xmm4
 
G_M000_IG27:                ;; offset=0x1C6C
       test     eax, eax
       sete     al
       movzx    rax, al
       test     ecx, ecx
       sete     cl
       movzx    rcx, cl
       test     eax, ecx
       je       G_M000_IG29
       vmovaps  xmm5, xmm2
       vxorps   xmm1, xmm5, xmmword ptr [reloc @RWD16]
       vmovss   xmm4, dword ptr [rbp-0x120]
       vmulss   xmm4, xmm4, dword ptr [rbp-0x120]
       vmovss   xmm5, dword ptr [rbp-0x118]
       vmulss   xmm5, xmm5, dword ptr [rbp-0x118]
       vaddss   xmm4, xmm4, xmm5
       vdivss   xmm4, xmm6, xmm4
       vmovss   xmm5, dword ptr [rsi+0x04]
       vmovss   xmm14, dword ptr [rdi+0x04]
       vmovups  xmm15, xmmword ptr [rbp-0xDC]
       vinsertps xmm12, xmm15, xmm15, 56
       vmovaps  xmm11, xmm1
       vinsertps xmm11, xmm11, xmm11, 56
       vmulps   xmm11, xmm11, xmm12
       vpermilps xmm12, xmm11, -11
       vaddps   xmm12, xmm12, xmm11
       vpermilps xmm11, xmm11, -86
       vaddps   xmm11, xmm11, xmm12
       vmovshdup xmm1, xmm1
       vmovss   xmm12, dword ptr [rbp-0xD8]
       vmulss   xmm13, xmm1, xmm12
       vsubss   xmm11, xmm11, xmm13
       vmulss   xmm13, xmm12, xmm12
       vsubss   xmm13, xmm6, xmm13
       vbroadcastss xmm15, dword ptr [reloc @RWD148]
       vmaxss   xmm13, xmm15, xmm13
       vdivss   xmm11, xmm11, xmm13
       vmulss   xmm11, xmm11, xmm12
       vsubss   xmm11, xmm11, xmm1
       vandps   xmm12, xmm12, xmmword ptr [reloc @RWD48]
       vmulss   xmm5, xmm12, xmm5
       vxorps   xmm12, xmm14, xmmword ptr [reloc @RWD16]
       vxorps   xmm13, xmm5, xmmword ptr [reloc @RWD16]
       vsubss   xmm13, xmm13, xmm1
       vmovaps  xmm15, xmm14
       vminss   xmm13, xmm15, xmm13
       vmaxss   xmm12, xmm12, xmm13
       vxorps   xmm13, xmm14, xmmword ptr [reloc @RWD16]
       vsubss   xmm1, xmm5, xmm1
       vmaxss   xmm1, xmm13, xmm1
       vminss   xmm1, xmm14, xmm1
       vmaxss   xmm5, xmm11, xmm12
       vminss   xmm11, xmm5, xmm1
       vmovss   xmm5, dword ptr [rbp-0xDC]
       vmulss   xmm5, xmm5, dword ptr [rbp-0x118]
       vmovss   xmm13, dword ptr [rbp-0xD4]
       vmulss   xmm13, xmm13, dword ptr [rbp-0x120]
       vsubss   xmm5, xmm5, xmm13
       vmulss   xmm5, xmm5, xmm5
       vmulss   xmm4, xmm5, xmm4
       vmovss   xmm5, dword ptr [reloc @RWD152]
 
G_M000_IG28:                ;; offset=0x1DB2
       vsubss   xmm4, xmm5, xmm4
       vmulss   xmm4, xmm4, dword ptr [reloc @RWD156]
       vbroadcastss xmm5, dword ptr [reloc @RWD00]
       vminss   xmm4, xmm5, xmm4
       vxorps   xmm5, xmm5, xmm5
       vmaxss   xmm4, xmm5, xmm4
       vmulss   xmm5, xmm11, xmm4
       vsubss   xmm5, xmm11, xmm5
       vmulss   xmm11, xmm4, xmm12
       vaddss   xmm11, xmm11, xmm5
       vmulss   xmm1, xmm4, xmm1
       vaddss   xmm1, xmm1, xmm5
       vmovaps  xmm4, xmm3
       vinsertps xmm4, xmm4, xmm11, 16
       vmovshdup xmm5, xmm3
       vinsertps xmm14, xmm4, xmm5, 40
       vmovaps  xmm4, xmm3
       vinsertps xmm4, xmm4, xmm1, 16
       vmovshdup xmm3, xmm3
       vinsertps xmm15, xmm4, xmm3, 40
       mov      byte  ptr [rbx+0x5C], 1
       vucomiss xmm1, xmm11
       seta     al
       mov      byte  ptr [rbx+0x5D], al
       vmovaps  xmm0, xmmword ptr [rbp-0x1A0]
       vmovaps  xmm5, xmm0
       vmovaps  xmm12, xmm2
 
G_M000_IG29:                ;; offset=0x1E33
       vmovaps  xmm0, xmm5
       vinsertps xmm0, xmm0, xmm0, 56
       vmovaps  xmm1, xmmword ptr [rbp-0x120]
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm0, xmm1, xmm0
       vpermilps xmm1, xmm0, -11
       vaddps   xmm1, xmm1, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm1
       vdivss   xmm0, xmm6, xmm0
       vmovss   xmm2, dword ptr [rbp+0x20]
       vxorps   xmm1, xmm2, xmmword ptr [reloc @RWD16]
       lea      rax, bword ptr [rbx+0x3C]
       lea      rcx, bword ptr [rbx+0x5C]
       vmovaps  xmm2, xmm14
       vmovaps  xmm3, xmm12
       vsubps   xmm2, xmm2, xmm3
       vinsertps xmm2, xmm2, xmm2, 56
       vmovaps  xmm3, xmm5
       vinsertps xmm3, xmm3, xmm3, 56
       vmulps   xmm2, xmm3, xmm2
       vpermilps xmm3, xmm2, -11
       vaddps   xmm3, xmm3, xmm2
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm3
       vmulss   xmm2, xmm2, xmm0
       vmovss   dword ptr [rax], xmm2
       vmovaps  xmm8, xmmword ptr [rbp-0x100]
       vmovaps  xmm2, xmm8
       vaddps   xmm2, xmm2, xmm14
       vmovaps  xmm3, xmm9
       vmovaps  xmm4, xmm2
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm10
       vmovshdup xmm6, xmm2
       vbroadcastss xmm6, xmm6
       vmulps   xmm4, xmm6, xmm4
       vaddps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm7
       vunpckhps xmm2, xmm2, xmm2
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm4
       vaddps   xmm2, xmm2, xmm3
       vmovsd   qword ptr [rbx], xmm2
       vextractps dword ptr [rbx+0x08], xmm2, 2
       vmovss   xmm2, dword ptr [rax]
       vucomiss xmm2, xmm1
       setae    al
       movzx    rax, al
       and      byte  ptr [rcx], al
       lea      rax, bword ptr [rbx+0x0C]
       lea      rcx, bword ptr [rbx+0x40]
       lea      rdx, bword ptr [rbx+0x5D]
       vmovaps  xmm2, xmm15
       vmovaps  xmm3, xmm12
       vsubps   xmm2, xmm2, xmm3
       vinsertps xmm2, xmm2, xmm2, 56
       vmovaps  xmm3, xmm5
       vinsertps xmm3, xmm3, xmm3, 56
       vmulps   xmm2, xmm3, xmm2
       vpermilps xmm3, xmm2, -11
       vaddps   xmm3, xmm3, xmm2
 
G_M000_IG30:                ;; offset=0x1F5C
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm3
       vmulss   xmm2, xmm2, xmm0
       vmovss   dword ptr [rcx], xmm2
       vmovaps  xmm2, xmm8
       vaddps   xmm2, xmm2, xmm15
       vmovaps  xmm3, xmm9
       vmovaps  xmm4, xmm2
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm10
       vmovshdup xmm6, xmm2
       vbroadcastss xmm6, xmm6
       vmulps   xmm4, xmm6, xmm4
       vaddps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm7
       vunpckhps xmm2, xmm2, xmm2
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm4
       vaddps   xmm2, xmm2, xmm3
       vmovsd   qword ptr [rax], xmm2
       vextractps dword ptr [rax+0x08], xmm2, 2
       vmovss   xmm2, dword ptr [rcx]
       vucomiss xmm2, xmm1
       setae    al
       movzx    rax, al
       and      byte  ptr [rdx], al
       lea      rax, bword ptr [rbx+0x18]
       lea      rcx, bword ptr [rbx+0x44]
       lea      rdx, bword ptr [rbx+0x5E]
       vmovaps  xmm2, xmmword ptr [rbp-0x160]
       vmovaps  xmm3, xmm2
       vmovaps  xmm4, xmm12
       vsubps   xmm3, xmm3, xmm4
       vinsertps xmm3, xmm3, xmm3, 56
       vmovaps  xmm4, xmm5
       vinsertps xmm4, xmm4, xmm4, 56
       vmulps   xmm3, xmm4, xmm3
       vpermilps xmm4, xmm3, -11
       vaddps   xmm4, xmm4, xmm3
       vpermilps xmm3, xmm3, -86
       vaddps   xmm3, xmm3, xmm4
       vmulss   xmm3, xmm3, xmm0
       vmovss   dword ptr [rcx], xmm3
       vmovaps  xmm3, xmm8
       vaddps   xmm2, xmm3, xmm2
       vmovaps  xmm3, xmm9
       vmovaps  xmm4, xmm2
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm10
       vmovshdup xmm6, xmm2
       vbroadcastss xmm6, xmm6
       vmulps   xmm4, xmm6, xmm4
       vaddps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm7
       vunpckhps xmm2, xmm2, xmm2
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm4
       vaddps   xmm2, xmm2, xmm3
       vmovsd   qword ptr [rax], xmm2
       vextractps dword ptr [rax+0x08], xmm2, 2
       vmovss   xmm2, dword ptr [rcx]
       vucomiss xmm2, xmm1
       setae    al
       movzx    rax, al
       and      byte  ptr [rdx], al
 
G_M000_IG31:                ;; offset=0x2082
       lea      rax, bword ptr [rbx+0x24]
       lea      rcx, bword ptr [rbx+0x48]
       lea      rdx, bword ptr [rbx+0x5F]
       vmovaps  xmm2, xmmword ptr [rbp-0x170]
       vmovaps  xmm3, xmm2
       vsubps   xmm3, xmm3, xmm12
       vinsertps xmm3, xmm3, xmm3, 56
       vinsertps xmm4, xmm5, xmm5, 56
       vmulps   xmm3, xmm4, xmm3
       vpermilps xmm4, xmm3, -11
       vaddps   xmm4, xmm4, xmm3
       vpermilps xmm3, xmm3, -86
       vaddps   xmm3, xmm3, xmm4
       vmulss   xmm0, xmm3, xmm0
       vmovss   dword ptr [rcx], xmm0
       vaddps   xmm0, xmm8, xmm2
       vmovaps  xmm2, xmm9
       vmovaps  xmm3, xmm0
       vbroadcastss xmm3, xmm3
       vmulps   xmm2, xmm3, xmm2
       vmovaps  xmm3, xmm10
       vmovshdup xmm4, xmm0
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vaddps   xmm2, xmm3, xmm2
       vmovaps  xmm3, xmm7
       vunpckhps xmm0, xmm0, xmm0
       vbroadcastss xmm0, xmm0
       vmulps   xmm0, xmm0, xmm3
       vaddps   xmm0, xmm0, xmm2
       vmovsd   qword ptr [rax], xmm0
       vextractps dword ptr [rax+0x08], xmm0, 2
       vmovss   xmm0, dword ptr [rcx]
       vucomiss xmm0, xmm1
       setae    al
       movzx    rax, al
       and      byte  ptr [rdx], al
       vmovaps  xmm0, xmmword ptr [rbp-0x120]
       lea      rax, bword ptr [rbx+0x30]
       vmovaps  xmm1, xmm0
       vbroadcastss xmm1, xmm1
       vmulps   xmm1, xmm1, xmm9
       vmovshdup xmm2, xmm0
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm10
       vaddps   xmm1, xmm2, xmm1
       vunpckhps xmm0, xmm0, xmm0
       vbroadcastss xmm0, xmm0
       vmulps   xmm0, xmm0, xmm7
       vaddps   xmm0, xmm0, xmm1
       vmovsd   qword ptr [rax], xmm0
       vextractps dword ptr [rax+0x08], xmm0, 2
       xor      eax, eax
       mov      dword ptr [rbx+0x4C], eax
       mov      dword ptr [rbx+0x50], 1
       mov      dword ptr [rbx+0x54], 2
       mov      dword ptr [rbx+0x58], 3
 
G_M000_IG32:                ;; offset=0x2189
       vmovaps  xmm6, xmmword ptr [rsp+0x590]
       vmovaps  xmm7, xmmword ptr [rsp+0x580]
       vmovaps  xmm8, xmmword ptr [rsp+0x570]
       vmovaps  xmm9, xmmword ptr [rsp+0x560]
       vmovaps  xmm10, xmmword ptr [rsp+0x550]
       vmovaps  xmm11, xmmword ptr [rsp+0x540]
       vmovaps  xmm12, xmmword ptr [rsp+0x530]
       vmovaps  xmm13, xmmword ptr [rsp+0x520]
       vmovaps  xmm14, xmmword ptr [rsp+0x510]
       vmovaps  xmm15, xmmword ptr [rsp+0x500]
       add      rsp, 0x5A8
       pop      rbx
       pop      rsi
       pop      rdi
       pop      rbp
       ret      
 
RWD00  	dd	3F800000h		;         1
RWD04  	dd	00000000h, 00000000h, 00000000h
RWD16  	dq	8000000080000000h, 8000000080000000h
RWD32  	dd	2EDBE6FFh		;     1e-10
RWD36  	dd	358637BDh		;     1e-06
RWD40  	dd	00000000h, 00000000h
RWD48  	dq	7FFFFFFF7FFFFFFFh, 7FFFFFFF7FFFFFFFh
RWD64  	dd	3F3504F3h		;  0.707107
RWD68  	dd	3F7FF972h		;    0.9999
RWD72  	dd	00000000h, 00000000h
RWD80  	dq	283424DC283424DCh, 283424DC283424DCh
RWD96  	dq	000000003F800000h, 0000000000000000h
RWD112 	dd	3F000000h		;       0.5
RWD116 	dd	00000000h, 00000000h, 00000000h
RWD128 	dq	2B8CBCCC2B8CBCCCh, 2B8CBCCC2B8CBCCCh
RWD144 	dd	469C395Dh		;   19996.7
RWD148 	dd	26901D7Dh		;     1e-15
RWD152 	dd	3CB851ECh		;    0.0225
RWD156 	dd	4234FED7h		;   45.2489

; Total bytes of code 8687

; Assembly listing for method AosBaselines.ScalarDepthRefiner`4[BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar,BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar]:FindMinimumDepth(byref,byref,byref,byref,byref,float,float,byref,byref,byref,int) (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rbp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 114 single block inlinees; 22 inlinees without PGO data

G_M000_IG01:                ;; offset=0x0000
       push     rbp
       push     r15
       push     r14
       push     r13
       push     r12
       push     rdi
       push     rsi
       push     rbx
       sub      rsp, 504
       vmovaps  xmmword ptr [rsp+0x1E0], xmm6
       vmovaps  xmmword ptr [rsp+0x1D0], xmm7
       vmovaps  xmmword ptr [rsp+0x1C0], xmm8
       vmovaps  xmmword ptr [rsp+0x1B0], xmm9
       vmovaps  xmmword ptr [rsp+0x1A0], xmm10
       vmovaps  xmmword ptr [rsp+0x190], xmm11
       vmovaps  xmmword ptr [rsp+0x180], xmm12
       vmovaps  xmmword ptr [rsp+0x170], xmm13
       vmovaps  xmmword ptr [rsp+0x160], xmm14
       vmovaps  xmmword ptr [rsp+0x150], xmm15
       lea      rbp, [rsp+0x230]
       vxorps   xmm4, xmm4, xmm4
       vmovdqa  xmmword ptr [rbp-0x1B0], xmm4
       mov      rax, -192
       vmovdqa  xmmword ptr [rbp+rax-0xE0], xmm4
       vmovdqa  xmmword ptr [rbp+rax-0xD0], xmm4
       vmovdqa  xmmword ptr [rbp+rax-0xC0], xmm4
       add      rax, 48
       jne      SHORT  -5 instr
       mov      rsi, rcx
       mov      rdi, rdx
       mov      rcx, bword ptr [rbp+0x30]
       vmovss   xmm6, dword ptr [rbp+0x38]
       mov      rbx, bword ptr [rbp+0x48]
       mov      r14, bword ptr [rbp+0x50]
       mov      r13, bword ptr [rbp+0x58]
       mov      r15d, dword ptr [rbp+0x60]
 
G_M000_IG02:                ;; offset=0x00CB
       vmovsd   xmm7, qword ptr [r9]
       vinsertps xmm7, xmm7, dword ptr [r9+0x08], 40
       vmovsd   xmm8, qword ptr [r9+0x0C]
       vinsertps xmm8, xmm8, dword ptr [r9+0x14], 40
       vmovsd   xmm9, qword ptr [r9+0x18]
       vinsertps xmm9, xmm9, dword ptr [r9+0x20], 40
       vmovshdup xmm0, xmm7
       vunpckhps xmm1, xmm7, xmm7
       vunpckhps xmm2, xmm8, xmm8
       vmovaps  xmm3, xmm7
       vmovaps  xmm4, xmm8
       vinsertps xmm3, xmm3, xmm4, 16
       vmovaps  xmm4, xmm9
       vinsertps xmm10, xmm3, xmm4, 40
       vmovaps  xmmword ptr [rbp-0x1C0], xmm10
       vmovshdup xmm3, xmm8
       vinsertps xmm0, xmm0, xmm3, 16
       vmovshdup xmm3, xmm9
       vinsertps xmm11, xmm0, xmm3, 40
       vinsertps xmm0, xmm1, xmm2, 16
       vunpckhps xmm1, xmm9, xmm9
       vinsertps xmm12, xmm0, xmm1, 40
       vmovsd   xmm13, qword ptr [r8]
       vinsertps xmm13, xmm13, dword ptr [r8+0x08], 40
       vmovsd   xmm0, qword ptr [rcx]
       vinsertps xmm0, xmm0, dword ptr [rcx+0x08], 40
       vmovshdup xmm1, xmm0
       vbroadcastss xmm1, xmm1
       vxorps   xmm2, xmm2, xmm2
       vcmpgtps xmm1, xmm1, xmm2
       vmovss   xmm2, dword ptr [rsi+0x04]
       vmovaps  xmm3, xmm2
       vxorps   xmm2, xmm2, xmmword ptr [reloc @RWD00]
       vblendvps xmm1, xmm2, xmm3, xmm1
       vmovaps  xmm2, xmm0
       vmovaps  xmm3, xmm0
       vmulss   xmm2, xmm2, xmm3
       vunpckhps xmm3, xmm0, xmm0
       vmulss   xmm4, xmm3, xmm3
       vaddss   xmm2, xmm2, xmm4
       vsqrtss  xmm2, xmm2, xmm2
       vmovss   xmm4, dword ptr [rsi]
       vdivss   xmm4, xmm4, xmm2
       vbroadcastss xmm2, xmm2
       vbroadcastss xmm14, dword ptr [reloc @RWD16]
       vcmpgtps xmm2, xmm2, xmm14
       vmulss   xmm0, xmm0, xmm4
       vinsertps xmm0, xmm0, xmm1, 16
       vmulss   xmm3, xmm3, xmm4
       vinsertps xmm0, xmm0, xmm3, 40
       vandps   xmm0, xmm0, xmm2
       vinsertps xmm1, xmm1, xmm1, 29
       vandnps  xmm1, xmm2, xmm1
       vorps    xmm0, xmm1, xmm0
       vmovsd   xmm1, qword ptr [rcx]
       vinsertps xmm1, xmm1, dword ptr [rcx+0x08], 40
       vmovaps  xmmword ptr [rbp-0x1F0], xmm1
       vbroadcastss xmm15, dword ptr [reloc @RWD00]
 
G_M000_IG03:                ;; offset=0x0201
       vxorps   xmm2, xmm15, xmm1
       vmovaps  xmm3, xmm10
       vmovaps  xmm4, xmm2
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm11
       vmovshdup xmm5, xmm2
       vbroadcastss xmm5, xmm5
       vmulps   xmm4, xmm5, xmm4
       vaddps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm12
       vunpckhps xmm2, xmm2, xmm2
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm4
       vaddps   xmm2, xmm2, xmm3
       vmovshdup xmm3, xmm2
       vbroadcastss xmm3, xmm3
       vxorps   xmm4, xmm4, xmm4
       vcmpgtps xmm3, xmm3, xmm4
       vmovss   xmm4, dword ptr [rdi+0x04]
       vmovaps  xmm5, xmm4
       vxorps   xmm1, xmm4, xmmword ptr [reloc @RWD00]
       vblendvps xmm1, xmm1, xmm5, xmm3
       vmovaps  xmm3, xmm2
       vmovaps  xmm4, xmm2
       vmulss   xmm3, xmm3, xmm4
       vunpckhps xmm4, xmm2, xmm2
       vmulss   xmm5, xmm4, xmm4
       vaddss   xmm3, xmm3, xmm5
       vsqrtss  xmm3, xmm3, xmm3
       vmovss   xmm5, dword ptr [rdi]
       vdivss   xmm5, xmm5, xmm3
       vbroadcastss xmm3, xmm3
       vcmpgtps xmm3, xmm3, xmm14
       vmulss   xmm2, xmm2, xmm5
       vinsertps xmm2, xmm2, xmm1, 16
       vmulss   xmm4, xmm4, xmm5
       vinsertps xmm2, xmm2, xmm4, 40
       vandps   xmm2, xmm2, xmm3
       vinsertps xmm1, xmm1, xmm1, 29
       vandnps  xmm1, xmm3, xmm1
       vorps    xmm1, xmm1, xmm2
       vmovaps  xmm2, xmm7
       vmovaps  xmm3, xmm1
       vbroadcastss xmm3, xmm3
       vmulps   xmm2, xmm3, xmm2
       vmovaps  xmm3, xmm8
       vmovshdup xmm4, xmm1
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vaddps   xmm2, xmm3, xmm2
       vmovaps  xmm3, xmm9
       vunpckhps xmm1, xmm1, xmm1
       vbroadcastss xmm1, xmm1
       vmulps   xmm1, xmm1, xmm3
       vaddps   xmm1, xmm1, xmm2
       vmovaps  xmm2, xmm13
       vaddps   xmm1, xmm2, xmm1
       vmovaps  xmm2, xmm0
       vsubps   xmm1, xmm2, xmm1
       vmovaps  xmm2, xmm1
       vinsertps xmm2, xmm2, xmm2, 56
       vmovaps  xmm3, xmmword ptr [rbp-0x1F0]
       vinsertps xmm3, xmm3, xmm3, 56
       vmulps   xmm2, xmm3, xmm2
       vpermilps xmm3, xmm2, -11
 
G_M000_IG04:                ;; offset=0x0331
       vaddps   xmm3, xmm3, xmm2
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm3
       vmovaps  xmm3, xmm1
       vmovups  xmmword ptr [rbp-0x180], xmm3
       vmovups  xmmword ptr [rbp-0x150], xmm3
       vmovups  xmmword ptr [rbp-0x120], xmm3
       vmovups  xmmword ptr [rbp-0x170], xmm0
       vmovups  xmmword ptr [rbp-0x140], xmm0
       vmovups  xmmword ptr [rbp-0x110], xmm0
       mov      byte  ptr [rbp-0x15C], 1
       mov      byte  ptr [rbp-0x12C], 0
       mov      byte  ptr [rbp-0xFC], 0
       vmovss   xmm0, dword ptr [rbp+0x40]
       vmovss   dword ptr [rbp-0x184], xmm0
       vmovsd   xmm3, qword ptr [rcx]
       vinsertps xmm3, xmm3, dword ptr [rcx+0x08], 40
       vmovsd   qword ptr [r14], xmm3
       vextractps dword ptr [r14+0x08], xmm3, 2
       vmovss   dword ptr [rbx], xmm2
       vucomiss xmm0, xmm2
       ja       G_M000_IG13
 
G_M000_IG05:                ;; offset=0x03BA
       vmovsd   xmm2, qword ptr [r14]
       vinsertps xmm2, xmm2, dword ptr [r14+0x08], 40
       vmovss   xmm3, dword ptr [rbx]
       vxorps   xmm4, xmm4, xmm4
       vmaxss   xmm4, xmm4, xmm3
       vbroadcastss xmm4, xmm4
       vmulps   xmm2, xmm4, xmm2
       vbroadcastss xmm4, xmm3
       vxorps   xmm5, xmm5, xmm5
       vcmpltps xmm4, xmm4, xmm5
       vsubss   xmm3, xmm6, xmm3
       vmovss   dword ptr [rbp+0x38], xmm6
       vmovaps  xmm5, xmm6
       vblendvps xmm3, xmm5, xmm3, xmm4
       vmulss   xmm3, xmm3, xmm3
       vmovss   dword ptr [rbp-0x188], xmm3
       vsubps   xmm1, xmm1, xmm2
       mov      dword ptr [rbp-0x160], 0x3F800000
       xor      r9d, r9d
       mov      dword ptr [rbp-0x130], r9d
 
G_M000_IG06:                ;; offset=0x0420
       mov      dword ptr [rbp-0x100], r9d
       mov      dword ptr [rbp-0xF0], 0x3F800000
       vmovaps  xmm4, xmm1
       vmovaps  xmm5, xmm1
       vmulss   xmm4, xmm4, xmm5
       vmovshdup xmm5, xmm1
       vmovshdup xmm3, xmm1
       vmulss   xmm3, xmm5, xmm3
       vaddss   xmm3, xmm4, xmm3
       vunpckhps xmm4, xmm1, xmm1
       vunpckhps xmm5, xmm1, xmm1
       vmulss   xmm4, xmm4, xmm5
       vaddss   xmm3, xmm3, xmm4
       vmovss   xmm4, dword ptr [rbp-0x188]
       vucomiss xmm4, xmm3
       seta     r9b
       movzx    r9, r9b
       mov      dword ptr [rbp-0x190], r9d
       vxorps   xmm3, xmm3, xmm3
       vmovaps  xmmword ptr [rbp-0x1A0], xmm3
       cmp      byte  ptr [rbp-0x190], 0
       jne      SHORT G_M000_IG07
       vxorps   xmm1, xmm1, xmm15
       vbroadcastss xmm3, dword ptr [rbx]
       vxorps   xmm4, xmm4, xmm4
       vcmpleps xmm3, xmm3, xmm4
       vmulps   xmm4, xmm1, xmmword ptr [reloc @RWD32]
       vaddps   xmm2, xmm4, xmm2
       vblendvps xmm1, xmm2, xmm1, xmm3
       vmovaps  xmm2, xmm1
       vmovaps  xmm3, xmm1
       vmulss   xmm2, xmm2, xmm3
       vmovshdup xmm3, xmm1
       vmovshdup xmm4, xmm1
       vmulss   xmm3, xmm3, xmm4
       vaddss   xmm2, xmm2, xmm3
       vunpckhps xmm3, xmm1, xmm1
       vunpckhps xmm4, xmm1, xmm1
       vmulss   xmm3, xmm3, xmm4
       vaddss   xmm2, xmm2, xmm3
       vsqrtss  xmm2, xmm2, xmm2
       vmovss   xmm3, dword ptr [reloc @RWD48]
       vdivss   xmm2, xmm3, xmm2
       vbroadcastss xmm2, xmm2
       vmulps   xmm1, xmm2, xmm1
       vmovaps  xmmword ptr [rbp-0x1A0], xmm1
 
G_M000_IG07:                ;; offset=0x04FF
       xor      r12d, r12d
       cmp      r12d, r15d
       jge      G_M000_IG12
 
G_M000_IG08:                ;; offset=0x050B
       cmp      byte  ptr [rbp-0x190], 0
       jne      G_M000_IG12
       vbroadcastss xmm1, dword ptr [rbp-0x19C]
       vxorps   xmm2, xmm2, xmm2
       vcmpgtps xmm1, xmm1, xmm2
       vmovss   xmm2, dword ptr [rsi+0x04]
       vmovaps  xmm3, xmm2
       vxorps   xmm2, xmm2, xmmword ptr [reloc @RWD00]
       vblendvps xmm1, xmm2, xmm3, xmm1
       vmovss   xmm2, dword ptr [rbp-0x1A0]
       vmulss   xmm3, xmm2, xmm2
       vmovss   xmm4, dword ptr [rbp-0x198]
       vmulss   xmm5, xmm4, xmm4
       vaddss   xmm3, xmm3, xmm5
       vsqrtss  xmm3, xmm3, xmm3
       vmovss   xmm5, dword ptr [rsi]
       vdivss   xmm5, xmm5, xmm3
       vbroadcastss xmm3, xmm3
       vcmpgtps xmm3, xmm3, xmm14
       vmulss   xmm2, xmm2, xmm5
       vinsertps xmm2, xmm2, xmm1, 16
       vmulss   xmm4, xmm4, xmm5
       vinsertps xmm2, xmm2, xmm4, 40
       vandps   xmm2, xmm2, xmm3
       vinsertps xmm1, xmm1, xmm1, 29
       vandnps  xmm1, xmm3, xmm1
       vorps    xmm1, xmm1, xmm2
       vmovaps  xmm2, xmmword ptr [rbp-0x1A0]
       vxorps   xmm3, xmm15, xmm2
       vmovaps  xmm4, xmm10
       vmovaps  xmm5, xmm3
       vbroadcastss xmm5, xmm5
       vmulps   xmm4, xmm5, xmm4
       vmovaps  xmm5, xmm11
       vmovshdup xmm6, xmm3
       vbroadcastss xmm6, xmm6
       vmulps   xmm5, xmm6, xmm5
       vaddps   xmm4, xmm5, xmm4
       vmovaps  xmm5, xmm12
       vunpckhps xmm3, xmm3, xmm3
       vbroadcastss xmm3, xmm3
       vmulps   xmm3, xmm3, xmm5
       vaddps   xmm3, xmm3, xmm4
       vmovshdup xmm4, xmm3
       vbroadcastss xmm4, xmm4
       vxorps   xmm5, xmm5, xmm5
       vcmpgtps xmm4, xmm4, xmm5
       vmovss   xmm5, dword ptr [rdi+0x04]
       vmovaps  xmm6, xmm5
       vxorps   xmm5, xmm5, xmmword ptr [reloc @RWD00]
       vblendvps xmm4, xmm5, xmm6, xmm4
       vmovaps  xmm5, xmm3
       vmulss   xmm6, xmm5, xmm5
       vunpckhps xmm3, xmm3, xmm3
       vmulss   xmm10, xmm3, xmm3
       vaddss   xmm6, xmm6, xmm10
       vsqrtss  xmm6, xmm6, xmm6
       vmovss   xmm10, dword ptr [rdi]
       vdivss   xmm10, xmm10, xmm6
       vbroadcastss xmm6, xmm6
       vcmpgtps xmm6, xmm6, xmm14
 
G_M000_IG09:                ;; offset=0x0639
       vmulss   xmm5, xmm5, xmm10
       vinsertps xmm5, xmm5, xmm4, 16
       vmulss   xmm3, xmm3, xmm10
       vinsertps xmm3, xmm5, xmm3, 40
       vandps   xmm3, xmm3, xmm6
       vinsertps xmm4, xmm4, xmm4, 29
       vandnps  xmm4, xmm6, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm7
       vmovaps  xmm5, xmm3
       vbroadcastss xmm5, xmm5
       vmulps   xmm4, xmm5, xmm4
       vmovaps  xmm5, xmm8
       vmovshdup xmm6, xmm3
       vbroadcastss xmm6, xmm6
       vmulps   xmm5, xmm6, xmm5
       vaddps   xmm4, xmm5, xmm4
       vmovaps  xmm5, xmm9
       vunpckhps xmm3, xmm3, xmm3
       vbroadcastss xmm3, xmm3
       vmulps   xmm3, xmm3, xmm5
       vaddps   xmm3, xmm3, xmm4
       vmovaps  xmm4, xmm13
       vaddps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm1
       vsubps   xmm3, xmm4, xmm3
       vmovaps  xmmword ptr [rbp-0x1B0], xmm3
       vmovaps  xmm3, xmmword ptr [rbp-0x1B0]
       vinsertps xmm3, xmm3, xmm3, 56
       vinsertps xmm2, xmm2, xmm2, 56
       vmulps   xmm2, xmm2, xmm3
       vpermilps xmm3, xmm2, -11
       vaddps   xmm3, xmm3, xmm2
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm3
       vmovss   xmm3, dword ptr [rbx]
       vucomiss xmm3, xmm2
       jbe      SHORT G_M000_IG11
 
G_M000_IG10:                ;; offset=0x06ED
       vmovss   dword ptr [rbx], xmm2
       vmovaps  xmm2, xmmword ptr [rbp-0x1A0]
       vmovsd   qword ptr [r14], xmm2
       vextractps dword ptr [r14+0x08], xmm2, 2
 
G_M000_IG11:                ;; offset=0x0705
       vucomiss xmm0, dword ptr [rbx]
       jae      G_M000_IG12
       vmovaps  xmmword ptr [rbp-0x1D0], xmm1
       lea      r9, [rbp-0x190]
       mov      qword ptr [rsp+0x20], r9
       vmovsd   xmm1, qword ptr [r14]
       vinsertps xmm1, xmm1, dword ptr [r14+0x08], 40
       vmovaps  xmmword ptr [rbp-0x1E0], xmm1
       vmovss   xmm1, dword ptr [rbx]
       vmovss   dword ptr [rsp+0x30], xmm1
       vmovss   xmm6, dword ptr [rbp+0x38]
       vmovss   dword ptr [rsp+0x38], xmm6
       lea      r9, [rbp-0x1D0]
       lea      rcx, [rbp-0x1E0]
       mov      qword ptr [rsp+0x28], rcx
       lea      rcx, [rbp-0x1A0]
       lea      rdx, [rbp-0x180]
       lea      r8, [rbp-0x1B0]
       call     [AosBaselines.ScalarDepthRefiner`4[BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar,BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar]:GetNextNormal(byref,System.Numerics.Vector3,System.Numerics.Vector3,byref,System.Numerics.Vector3,float,float):System.Numerics.Vector3]
       inc      r12d
       cmp      r12d, r15d
       vmovss   dword ptr [rbp+0x38], xmm6
       vmovss   xmm0, dword ptr [rbp-0x184]
       vmovaps  xmm10, xmmword ptr [rbp-0x1C0]
       jl       G_M000_IG08
 
G_M000_IG12:                ;; offset=0x079B
       vmovss   xmm0, dword ptr [reloc @RWD48]
       vdivss   xmm0, xmm0, dword ptr [rbp-0xF0]
       vmovups  xmm1, xmmword ptr [rbp-0x170]
       vmulss   xmm2, xmm0, dword ptr [rbp-0x160]
       vbroadcastss xmm2, xmm2
       vmulps   xmm1, xmm2, xmm1
       vmovups  xmm2, xmmword ptr [rbp-0x140]
       vmulss   xmm3, xmm0, dword ptr [rbp-0x130]
       vbroadcastss xmm3, xmm3
       vmulps   xmm2, xmm3, xmm2
       vmovups  xmm3, xmmword ptr [rbp-0x110]
       vmulss   xmm0, xmm0, dword ptr [rbp-0x100]
       vbroadcastss xmm0, xmm0
       vmulps   xmm0, xmm0, xmm3
       vaddps   xmm1, xmm2, xmm1
       vmovsd   qword ptr [r13], xmm1
       vextractps dword ptr [r13+0x08], xmm1, 2
       vmovsd   xmm1, qword ptr [r13]
       vinsertps xmm1, xmm1, dword ptr [r13+0x08], 40
       vaddps   xmm0, xmm1, xmm0
       vmovsd   qword ptr [r13], xmm0
       vextractps dword ptr [r13+0x08], xmm0, 2
       jmp      SHORT G_M000_IG14
 
G_M000_IG13:                ;; offset=0x0827
       vxorps   xmm0, xmm0, xmm0
       vmovsd   qword ptr [r13], xmm0
       vmovss   dword ptr [r13+0x08], xmm0
 
G_M000_IG14:                ;; offset=0x0837
       vmovaps  xmm6, xmmword ptr [rsp+0x1E0]
       vmovaps  xmm7, xmmword ptr [rsp+0x1D0]
       vmovaps  xmm8, xmmword ptr [rsp+0x1C0]
       vmovaps  xmm9, xmmword ptr [rsp+0x1B0]
       vmovaps  xmm10, xmmword ptr [rsp+0x1A0]
       vmovaps  xmm11, xmmword ptr [rsp+0x190]
       vmovaps  xmm12, xmmword ptr [rsp+0x180]
       vmovaps  xmm13, xmmword ptr [rsp+0x170]
       vmovaps  xmm14, xmmword ptr [rsp+0x160]
       vmovaps  xmm15, xmmword ptr [rsp+0x150]
       add      rsp, 504
       pop      rbx
       pop      rsi
       pop      rdi
       pop      r12
       pop      r13
       pop      r14
       pop      r15
       pop      rbp
       ret      
 
RWD00  	dq	8000000080000000h, 8000000080000000h
RWD16  	dd	322BCC77h		;     1e-08
RWD20  	dd	00000000h, 00000000h, 00000000h
RWD32  	dq	4080000040800000h, 4080000040800000h
RWD48  	dd	3F800000h		;         1

; Total bytes of code 2213

; Assembly listing for method AosBaselines.ScalarDepthRefiner`4[BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar,BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar]:GetNextNormal(byref,System.Numerics.Vector3,System.Numerics.Vector3,byref,System.Numerics.Vector3,float,float):System.Numerics.Vector3 (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rsp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 56 single block inlinees; 55 inlinees without PGO data

G_M000_IG01:                ;; offset=0x0000
       push     rdi
       push     rsi
       push     rbx
       sub      rsp, 352
       vmovaps  xmmword ptr [rsp+0x150], xmm6
       vmovaps  xmmword ptr [rsp+0x140], xmm7
       vmovaps  xmmword ptr [rsp+0x130], xmm8
       vmovaps  xmmword ptr [rsp+0x120], xmm9
       vmovaps  xmmword ptr [rsp+0x110], xmm10
       vmovaps  xmmword ptr [rsp+0x100], xmm11
       vmovaps  xmmword ptr [rsp+0xF0], xmm12
       vmovaps  xmmword ptr [rsp+0xE0], xmm13
       vmovaps  xmmword ptr [rsp+0xD0], xmm14
       vmovaps  xmmword ptr [rsp+0xC0], xmm15
       mov      rax, bword ptr [rsp+0x1A0]
       mov      r10, bword ptr [rsp+0x1A8]
       vmovss   xmm0, dword ptr [rsp+0x1B0]
       vmovss   xmm1, dword ptr [rsp+0x1B8]
 
G_M000_IG02:                ;; offset=0x0086
       vxorps   xmm2, xmm2, xmm2
       vmovaps  xmmword ptr [rsp+0xB0], xmm2
       vmovsd   xmm3, qword ptr [r10]
       vinsertps xmm3, xmm3, dword ptr [r10+0x08], 40
       vxorps   xmm4, xmm4, xmm4
       vmaxss   xmm4, xmm4, xmm0
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vbroadcastss xmm4, xmm0
       vxorps   xmm5, xmm5, xmm5
       vcmpltps xmm4, xmm4, xmm5
       vmovss   dword ptr [rsp+0x1B0], xmm0
       vsubss   xmm5, xmm1, xmm0
       vblendvps xmm1, xmm1, xmm5, xmm4
       vmulss   xmm1, xmm1, xmm1
       vmovss   dword ptr [rsp+0xAC], xmm1
       vmovsd   xmm4, qword ptr [r8]
       vinsertps xmm4, xmm4, dword ptr [r8+0x08], 40
       vmovsd   xmm5, qword ptr [r9]
       vinsertps xmm5, xmm5, dword ptr [r9+0x08], 40
       movzx    r9, byte  ptr [rdx+0x24]
       movzx    r11, byte  ptr [rdx+0x54]
       and      r11d, r9d
       movzx    rbx, byte  ptr [rdx+0x84]
       and      r11d, ebx
       mov      ebx, -1
       xor      esi, esi
       test     r9d, r9d
       cmovne   ebx, esi
       vmovd    xmm6, ebx
       vpbroadcastd xmm6, xmm6
       vandps   xmm7, xmm4, xmm6
       vandnps  xmm8, xmm6, xmmword ptr [rdx]
       vorps    xmm7, xmm8, xmm7
       vmovups  xmmword ptr [rdx], xmm7
       vandps   xmm7, xmm5, xmm6
       vandnps  xmm6, xmm6, xmmword ptr [rdx+0x10]
       vorps    xmm6, xmm6, xmm7
       vmovups  xmmword ptr [rdx+0x10], xmm6
       lea      r9, bword ptr [rdx+0x30]
       mov      ebx, -1
 
G_M000_IG03:                ;; offset=0x014E
       cmp      byte  ptr [r9+0x24], 0
       cmovne   ebx, esi
       vmovd    xmm6, ebx
       vpbroadcastd xmm6, xmm6
       vandps   xmm7, xmm4, xmm6
       vandnps  xmm8, xmm6, xmmword ptr [r9]
       vorps    xmm7, xmm8, xmm7
       vmovups  xmmword ptr [r9], xmm7
       vandps   xmm7, xmm5, xmm6
       vandnps  xmm6, xmm6, xmmword ptr [r9+0x10]
       vorps    xmm6, xmm6, xmm7
       vmovups  xmmword ptr [r9+0x10], xmm6
       lea      r9, bword ptr [rdx+0x60]
       mov      ebx, -1
 
G_M000_IG04:                ;; offset=0x018E
       cmp      byte  ptr [r9+0x24], 0
       cmovne   ebx, esi
       vmovd    xmm6, ebx
       vpbroadcastd xmm6, xmm6
       vandps   xmm7, xmm4, xmm6
       vandnps  xmm8, xmm6, xmmword ptr [r9]
       vorps    xmm7, xmm8, xmm7
       vmovups  xmmword ptr [r9], xmm7
       vandps   xmm7, xmm5, xmm6
       vandnps  xmm6, xmm6, xmmword ptr [r9+0x10]
       vorps    xmm6, xmm6, xmm7
       vmovups  xmmword ptr [r9+0x10], xmm6
       test     r11d, r11d
       je       G_M000_IG07
 
G_M000_IG05:                ;; offset=0x01CE
       vmovups  xmm6, xmmword ptr [rdx]
       vmovups  xmm7, xmmword ptr [rdx+0x30]
       vmovups  xmm8, xmmword ptr [rdx+0x60]
       vsubps   xmm9, xmm7, xmm6
       vsubps   xmm10, xmm6, xmm8
       vmovsd   xmm11, qword ptr [r8]
       vinsertps xmm11, xmm11, dword ptr [r8+0x08], 40
       vsubps   xmm6, xmm11, xmm6
       vsubps   xmm7, xmm11, xmm7
       vsubps   xmm8, xmm11, xmm8
       vpermilps xmm12, xmm9, 9
       vpermilps xmm13, xmm10, 18
       vmulps   xmm12, xmm13, xmm12
       vpermilps xmm9, xmm9, 18
       vpermilps xmm10, xmm10, 9
       vmulps   xmm9, xmm10, xmm9
       vsubps   xmm9, xmm12, xmm9
       vmovaps  xmm10, xmm3
       vsubps   xmm10, xmm11, xmm10
       vpermilps xmm11, xmm9, 9
       vpermilps xmm12, xmm10, 18
       vmulps   xmm11, xmm12, xmm11
       vpermilps xmm9, xmm9, 18
       vpermilps xmm10, xmm10, 9
       vmulps   xmm9, xmm10, xmm9
       vsubps   xmm9, xmm11, xmm9
       vinsertps xmm9, xmm9, xmm9, 56
       vinsertps xmm6, xmm6, xmm6, 56
       vmulps   xmm6, xmm6, xmm9
       vpermilps xmm10, xmm6, -11
       vaddps   xmm10, xmm10, xmm6
       vpermilps xmm6, xmm6, -86
       vaddps   xmm6, xmm6, xmm10
       vinsertps xmm7, xmm7, xmm7, 56
       vmulps   xmm7, xmm7, xmm9
       vpermilps xmm10, xmm7, -11
       vaddps   xmm10, xmm10, xmm7
       vpermilps xmm7, xmm7, -86
       vaddps   xmm7, xmm7, xmm10
       vinsertps xmm8, xmm8, xmm8, 56
       vmulps   xmm8, xmm8, xmm9
       vpermilps xmm9, xmm8, -11
       vaddps   xmm9, xmm9, xmm8
       vpermilps xmm8, xmm8, -86
       vaddps   xmm8, xmm8, xmm9
       vbroadcastss xmm6, xmm6
       vxorps   xmm9, xmm9, xmm9
       vcmpgeps xmm9, xmm6, xmm9
       vbroadcastss xmm7, xmm7
       vxorps   xmm10, xmm10, xmm10
       vcmpgeps xmm10, xmm7, xmm10
       vbroadcastss xmm8, xmm8
       vxorps   xmm11, xmm11, xmm11
       vcmpgeps xmm11, xmm8, xmm11
       vxorps   xmm12, xmm12, xmm12
       vcmpltps xmm7, xmm7, xmm12
       vcmpltps xmm8, xmm8, xmm12
       vandps   xmm8, xmm8, xmm10
 
G_M000_IG06:                ;; offset=0x0302
       vcmpltps xmm6, xmm6, xmm12
       vandps   xmm6, xmm6, xmm11
       vandps   xmm7, xmm9, xmm7
       vandps   xmm9, xmm4, xmm8
       vandnps  xmm10, xmm8, xmmword ptr [rdx]
       vorps    xmm9, xmm10, xmm9
       vmovups  xmmword ptr [rdx], xmm9
       vandps   xmm9, xmm5, xmm8
       vandnps  xmm10, xmm8, xmmword ptr [rdx+0x10]
       vorps    xmm9, xmm10, xmm9
       vmovups  xmmword ptr [rdx+0x10], xmm9
       lea      r8, bword ptr [rdx+0x30]
       vandps   xmm9, xmm4, xmm6
       vandnps  xmm10, xmm6, xmmword ptr [r8]
       vorps    xmm9, xmm10, xmm9
       vmovups  xmmword ptr [r8], xmm9
       vandps   xmm9, xmm5, xmm6
       vandnps  xmm10, xmm6, xmmword ptr [r8+0x10]
       vorps    xmm9, xmm10, xmm9
       vmovups  xmmword ptr [r8+0x10], xmm9
       vorps    xmm8, xmm7, xmm8
       vorps    xmm6, xmm8, xmm6
       vpcmpeqd xmm8, xmm8, xmm8
       vxorps   xmm6, xmm8, xmm6
       vorps    xmm6, xmm6, xmm7
       lea      r8, bword ptr [rdx+0x60]
       vandps   xmm4, xmm4, xmm6
       vandnps  xmm7, xmm6, xmmword ptr [r8]
       vorps    xmm4, xmm7, xmm4
       vmovups  xmmword ptr [r8], xmm4
       vandps   xmm4, xmm5, xmm6
       vandnps  xmm5, xmm6, xmmword ptr [r8+0x10]
       vorps    xmm4, xmm5, xmm4
       vmovups  xmmword ptr [r8+0x10], xmm4
 
G_M000_IG07:                ;; offset=0x03A3
       vmovups  xmm4, xmmword ptr [rdx]
       vmovups  xmm5, xmmword ptr [rdx+0x30]
       vmovups  xmm6, xmmword ptr [rdx+0x60]
       vmovaps  xmm7, xmm5
       vsubps   xmm7, xmm7, xmm4
       vsubps   xmm8, xmm4, xmm6
       vmovaps  xmm9, xmm5
       vsubps   xmm9, xmm6, xmm9
       vmovaps  xmm10, xmm8
       vmovaps  xmm11, xmm10
       vmovaps  xmm12, xmm7
       vmovaps  xmm13, xmm12
       vpermilps xmm14, xmm13, 9
       vpermilps xmm15, xmm11, 18
       vmulps   xmm14, xmm15, xmm14
       vpermilps xmm13, xmm13, 18
       vpermilps xmm11, xmm11, 9
       vmulps   xmm11, xmm11, xmm13
       vsubps   xmm11, xmm14, xmm11
       vmovaps  xmm13, xmm11
       vmovaps  xmm14, xmm11
       vmulss   xmm13, xmm13, xmm14
       vmovshdup xmm14, xmm11
       vmovshdup xmm15, xmm11
       vmulss   xmm14, xmm14, xmm15
       vaddss   xmm13, xmm13, xmm14
       vunpckhps xmm14, xmm11, xmm11
       vunpckhps xmm15, xmm11, xmm11
       vmulss   xmm14, xmm14, xmm15
       vaddss   xmm13, xmm13, xmm14
       vsubps   xmm4, xmm4, xmm3
       vsubps   xmm6, xmm6, xmm3
       vmovaps  xmmword ptr [rsp+0x90], xmm6
       vmovaps  xmm14, xmm4
       vmovaps  xmm15, xmm14
       vmovaps  xmmword ptr [rsp+0x20], xmm15
       vmovaps  xmm2, xmm12
       vpermilps xmm1, xmm2, 9
       vpermilps xmm15, xmm15, 18
       vmulps   xmm1, xmm15, xmm1
       vpermilps xmm2, xmm2, 18
       vpermilps xmm15, xmmword ptr [rsp+0x20], 9
       vmulps   xmm2, xmm15, xmm2
       vsubps   xmm1, xmm1, xmm2
       vmovaps  xmm2, xmm6
       vmovaps  xmmword ptr [rsp+0x10], xmm2
       vmovaps  xmm15, xmm10
       vpermilps xmm2, xmm15, 9
       vpermilps xmm0, xmmword ptr [rsp+0x10], 18
       vmulps   xmm0, xmm0, xmm2
       vpermilps xmm2, xmm15, 18
       vpermilps xmm15, xmmword ptr [rsp+0x10], 9
       vmulps   xmm2, xmm15, xmm2
       vsubps   xmm0, xmm0, xmm2
       vinsertps xmm1, xmm1, xmm1, 56
       vinsertps xmm2, xmm11, xmm11, 56
       vmulps   xmm1, xmm1, xmm2
       vpermilps xmm15, xmm1, -11
       vaddps   xmm15, xmm15, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm15
       vmovss   dword ptr [rsp+0x8C], xmm1
 
G_M000_IG08:                ;; offset=0x04E7
       vinsertps xmm0, xmm0, xmm0, 56
       vmulps   xmm0, xmm0, xmm2
       vpermilps xmm15, xmm0, -11
       vaddps   xmm15, xmm15, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm15
       vmovss   dword ptr [rsp+0x88], xmm0
       vsubss   xmm15, xmm13, xmm0
       vsubss   xmm15, xmm15, xmm1
       vmovss   dword ptr [rsp+0x84], xmm15
       vxorps   xmm15, xmm15, xmm15
       vucomiss xmm15, dword ptr [rsp+0x84]
       seta     r8b
       movzx    r8, r8b
       vucomiss xmm15, xmm0
       seta     r9b
       movzx    r9, r9b
       vmovaps  xmm15, xmm7
       vmovaps  xmm0, xmm7
       vmulss   xmm0, xmm15, xmm0
       vmovshdup xmm15, xmm7
       vmovshdup xmm6, xmm7
       vmulss   xmm6, xmm15, xmm6
       vaddss   xmm0, xmm0, xmm6
       vunpckhps xmm6, xmm7, xmm7
       vunpckhps xmm7, xmm7, xmm7
       vmulss   xmm6, xmm6, xmm7
       vaddss   xmm0, xmm0, xmm6
       vmovss   dword ptr [rsp+0x80], xmm0
       vmovaps  xmm6, xmm9
       vmovaps  xmm7, xmm9
       vmulss   xmm6, xmm6, xmm7
       vmovshdup xmm7, xmm9
       vmovshdup xmm15, xmm9
       vmulss   xmm7, xmm7, xmm15
       vaddss   xmm6, xmm6, xmm7
       vunpckhps xmm7, xmm9, xmm9
       vunpckhps xmm15, xmm9, xmm9
       vmulss   xmm7, xmm7, xmm15
       vaddss   xmm6, xmm6, xmm7
       vmovaps  xmm7, xmm8
       vmovaps  xmm15, xmm8
       vmulss   xmm7, xmm7, xmm15
       vmovshdup xmm15, xmm8
       vmovshdup xmm0, xmm8
       vmulss   xmm0, xmm15, xmm0
       vaddss   xmm0, xmm7, xmm0
       vunpckhps xmm7, xmm8, xmm8
       vunpckhps xmm8, xmm8, xmm8
       vmulss   xmm7, xmm7, xmm8
       vaddss   xmm0, xmm0, xmm7
       vmovss   xmm8, dword ptr [rsp+0x80]
       vmaxss   xmm8, xmm8, xmm6
       vmaxss   xmm8, xmm8, xmm0
       vmulss   xmm15, xmm8, dword ptr [reloc @RWD00]
       vucomiss xmm15, xmm13
       setae    r11b
       movzx    r11, r11b
       vmovss   xmm15, dword ptr [reloc @RWD04]
       vucomiss xmm15, xmm8
       seta     bl
       movzx    rbx, bl
       test     ebx, ebx
       sete     sil
       movzx    rsi, sil
       and      esi, r11d
       vmovsd   xmm8, qword ptr [r10]
 
G_M000_IG09:                ;; offset=0x062A
       vinsertps xmm8, xmm8, dword ptr [r10+0x08], 40
       vinsertps xmm8, xmm8, xmm8, 56
       vmulps   xmm2, xmm8, xmm2
       vpermilps xmm8, xmm2, -11
       vaddps   xmm8, xmm8, xmm2
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm8
       vbroadcastss xmm2, xmm2
       vxorps   xmm8, xmm8, xmm8
       vcmpltps xmm2, xmm2, xmm8
       vxorps   xmm8, xmm11, xmmword ptr [reloc @RWD16]
       vandps   xmm8, xmm8, xmm2
       vandnps  xmm2, xmm2, xmm11
       vorps    xmm11, xmm2, xmm8
       vxorps   xmm2, xmm14, xmmword ptr [reloc @RWD16]
       mov      r10d, 1
       vmovss   xmm8, dword ptr [reloc @RWD32]
       vmovss   dword ptr [rdx+0x20], xmm8
       xor      edi, edi
       mov      dword ptr [rdx+0x50], edi
       vxorps   xmm15, xmm15, xmm15
       vucomiss xmm15, xmm1
       seta     dil
       movzx    rdi, dil
       or       r8d, edi
       or       r8d, r9d
       xor      r9d, r9d
       mov      dword ptr [rdx+0x80], r9d
       vmovss   dword ptr [rdx+0x90], xmm8
       vmovaps  xmm15, xmm4
       vmovaps  xmm7, xmm4
       vmulss   xmm7, xmm15, xmm7
       vmovshdup xmm15, xmm4
       vmovshdup xmm1, xmm4
       vmulss   xmm1, xmm15, xmm1
       vaddss   xmm1, xmm7, xmm1
       vunpckhps xmm7, xmm4, xmm4
       vunpckhps xmm4, xmm4, xmm4
       vmulss   xmm4, xmm7, xmm4
       vaddss   xmm1, xmm1, xmm4
       vmovss   xmm4, dword ptr [rsp+0xAC]
       vucomiss xmm4, xmm1
       seta     r9b
       movzx    r9, r9b
       test     r9d, ebx
       je       SHORT G_M000_IG11
 
G_M000_IG10:                ;; offset=0x0705
       mov      byte  ptr [rax], 1
 
G_M000_IG11:                ;; offset=0x0708
       or       esi, r8d
       cmp      byte  ptr [rax], 0
       sete     r9b
       movzx    r9, r9b
       test     esi, r9d
       je       G_M000_IG17
 
G_M000_IG12:                ;; offset=0x071F
       vmovss   xmm7, dword ptr [rsp+0x80]
       vdivss   xmm2, xmm8, xmm7
       vmovss   dword ptr [rsp+0x7C], xmm2
       vdivss   xmm1, xmm8, xmm6
       vmovss   dword ptr [rsp+0x78], xmm1
       vdivss   xmm15, xmm8, xmm0
       vmovss   dword ptr [rsp+0x74], xmm15
       vsubps   xmm5, xmm5, xmm3
       vinsertps xmm15, xmm14, xmm14, 56
       vinsertps xmm1, xmm12, xmm12, 56
       vmulps   xmm1, xmm1, xmm15
       vpermilps xmm15, xmm1, -11
       vaddps   xmm15, xmm15, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm15
       vmovss   dword ptr [rsp+0x70], xmm1
       vmovaps  xmmword ptr [rsp], xmm5
       vinsertps xmm15, xmm5, xmm5, 56
       vmovaps  xmm2, xmm9
       vinsertps xmm2, xmm2, xmm2, 56
       vmulps   xmm2, xmm2, xmm15
       vpermilps xmm15, xmm2, -11
       vaddps   xmm15, xmm15, xmm2
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm15
       vmovss   dword ptr [rsp+0x6C], xmm2
       vmovaps  xmm15, xmmword ptr [rsp+0x90]
       vmovaps  xmm2, xmm15
       vinsertps xmm2, xmm2, xmm2, 56
       vinsertps xmm1, xmm10, xmm10, 56
       vmulps   xmm1, xmm1, xmm2
       vpermilps xmm2, xmm1, -11
       vaddps   xmm2, xmm2, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm2
       vmovss   dword ptr [rsp+0x68], xmm1
       vmovss   xmm1, dword ptr [rsp+0x70]
       vxorps   xmm1, xmm1, xmmword ptr [reloc @RWD16]
       vminss   xmm1, xmm7, xmm1
       vxorps   xmm2, xmm2, xmm2
       vmaxss   xmm1, xmm2, xmm1
       vmovss   xmm7, dword ptr [rsp+0x6C]
       vxorps   xmm2, xmm7, xmmword ptr [reloc @RWD16]
       vmovaps  xmm7, xmm6
       vminss   xmm2, xmm7, xmm2
       vxorps   xmm7, xmm7, xmm7
       vmaxss   xmm2, xmm7, xmm2
       vmulss   xmm1, xmm1, dword ptr [rsp+0x7C]
       vmovss   dword ptr [rsp+0x64], xmm1
       vmulss   xmm2, xmm2, dword ptr [rsp+0x78]
       vmovss   dword ptr [rsp+0x60], xmm2
       vmovss   xmm4, dword ptr [rsp+0x68]
       vxorps   xmm4, xmm4, xmmword ptr [reloc @RWD16]
       vmovaps  xmm7, xmm0
       vminss   xmm4, xmm7, xmm4
       vxorps   xmm7, xmm7, xmm7
       vmaxss   xmm4, xmm7, xmm4
       vmulss   xmm4, xmm4, dword ptr [rsp+0x74]
 
G_M000_IG13:                ;; offset=0x0858
       vmovss   dword ptr [rsp+0x5C], xmm4
       vbroadcastss xmm7, xmm1
       vmulps   xmm7, xmm7, xmm12
       vmovaps  xmmword ptr [rsp+0x40], xmm7
       vmovaps  xmm7, xmm9
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm7
       vbroadcastss xmm7, xmm4
       vmulps   xmm7, xmm7, xmm10
       vmovaps  xmmword ptr [rsp+0x30], xmm7
       vmovaps  xmm7, xmmword ptr [rsp+0x40]
       vaddps   xmm7, xmm7, xmm14
       vaddps   xmm2, xmm2, xmm5
       vmovaps  xmm4, xmm15
       vmovaps  xmm1, xmmword ptr [rsp+0x30]
       vaddps   xmm1, xmm1, xmm4
       vmovaps  xmm4, xmm7
       vmovaps  xmm15, xmm7
       vmulss   xmm4, xmm4, xmm15
       vmovshdup xmm15, xmm7
       vmovshdup xmm5, xmm7
       vmulss   xmm5, xmm15, xmm5
       vaddss   xmm4, xmm4, xmm5
       vunpckhps xmm5, xmm7, xmm7
       vunpckhps xmm7, xmm7, xmm7
       vmulss   xmm5, xmm5, xmm7
       vaddss   xmm4, xmm4, xmm5
       vmovaps  xmm5, xmm2
       vmovaps  xmm7, xmm2
       vmulss   xmm5, xmm5, xmm7
       vmovshdup xmm7, xmm2
       vmovshdup xmm15, xmm2
       vmulss   xmm7, xmm7, xmm15
       vaddss   xmm5, xmm5, xmm7
       vunpckhps xmm7, xmm2, xmm2
       vunpckhps xmm2, xmm2, xmm2
       vmulss   xmm2, xmm7, xmm2
       vaddss   xmm2, xmm5, xmm2
       vmovaps  xmm5, xmm1
       vmovaps  xmm7, xmm1
       vmulss   xmm5, xmm5, xmm7
       vmovshdup xmm7, xmm1
       vmovshdup xmm15, xmm1
       vmulss   xmm7, xmm7, xmm15
       vaddss   xmm5, xmm5, xmm7
       vunpckhps xmm7, xmm1, xmm1
       vunpckhps xmm1, xmm1, xmm1
       vmulss   xmm1, xmm7, xmm1
       vaddss   xmm1, xmm5, xmm1
       vbroadcastss xmm5, xmm6
       vxorps   xmm6, xmm6, xmm6
       vcmpeqps xmm5, xmm6, xmm5
       vbroadcastss xmm0, xmm0
       vcmpeqps xmm0, xmm6, xmm0
       vbroadcastss xmm6, xmm4
       vbroadcastss xmm7, xmm2
       vcmpltps xmm15, xmm6, xmm7
       vorps    xmm5, xmm15, xmm5
       vbroadcastss xmm15, xmm1
       vcmpltps xmm6, xmm6, xmm15
       vorps    xmm6, xmm6, xmm0
       vcmpltps xmm7, xmm7, xmm15
       vandps   xmm5, xmm5, xmm6
       vorps    xmm0, xmm7, xmm0
       vandnps  xmm0, xmm5, xmm0
       vandps   xmm4, xmm4, xmm5
 
G_M000_IG14:                ;; offset=0x0981
       vandps   xmm2, xmm2, xmm0
       vandnps  xmm1, xmm0, xmm1
       vorps    xmm1, xmm1, xmm2
       vandnps  xmm1, xmm5, xmm1
       vorps    xmm1, xmm1, xmm4
       vmovss   xmm4, dword ptr [rsp+0xAC]
       vucomiss xmm4, xmm1
       jb       SHORT G_M000_IG15
       mov      byte  ptr [rax], 1
 
G_M000_IG15:                ;; offset=0x09A7
       vmovss   xmm1, dword ptr [rsp+0x64]
       vandps   xmm1, xmm1, xmm5
       vmovss   xmm2, dword ptr [rsp+0x60]
       vandps   xmm2, xmm2, xmm0
       vmovss   xmm6, dword ptr [rsp+0x5C]
       vandnps  xmm6, xmm0, xmm6
       vorps    xmm2, xmm6, xmm2
       vandnps  xmm2, xmm5, xmm2
       vorps    xmm1, xmm2, xmm1
       vandps   xmm2, xmm9, xmm0
       vandps   xmm6, xmm12, xmm5
       vandnps  xmm7, xmm5, xmm10
       vorps    xmm6, xmm7, xmm6
       vandnps  xmm6, xmm0, xmm6
       vorps    xmm2, xmm6, xmm2
       vxorps   xmm6, xmm1, xmmword ptr [reloc @RWD16]
       vbroadcastss xmm6, xmm6
       vmulps   xmm2, xmm6, xmm2
       vmovaps  xmm6, xmmword ptr [rsp]
       vandps   xmm6, xmm6, xmm0
       vandps   xmm7, xmm14, xmm5
       vmovaps  xmm15, xmmword ptr [rsp+0x90]
       vandnps  xmm9, xmm5, xmm15
       vorps    xmm7, xmm9, xmm7
       vandnps  xmm7, xmm0, xmm7
       vorps    xmm6, xmm7, xmm6
       vsubps   xmm2, xmm2, xmm6
       vbroadcastss xmm6, xmm1
       vxorps   xmm7, xmm7, xmm7
       vcmpeqps xmm6, xmm7, xmm6
       vbroadcastss xmm7, xmm1
       vcmpeqps xmm7, xmm7, xmmword ptr [reloc @RWD48]
       vpand    xmm9, xmm6, xmmword ptr [reloc @RWD64]
       vpand    xmm10, xmm7, xmmword ptr [reloc @RWD80]
       vpandn   xmm12, xmm7, xmmword ptr [reloc @RWD96]
       vpor     xmm10, xmm12, xmm10
       vmovd    r10d, xmm10
       vmovd    xmm10, r10d
       vpandn   xmm10, xmm6, xmm10
       vpor     xmm9, xmm10, xmm9
       vmovd    r10d, xmm9
       vpand    xmm9, xmm6, xmmword ptr [reloc @RWD80]
       vpand    xmm10, xmm7, xmmword ptr [reloc @RWD112]
       vpandn   xmm12, xmm7, xmmword ptr [reloc @RWD128]
       vpor     xmm10, xmm12, xmm10
       vmovd    r9d, xmm10
       vmovd    xmm10, r9d
       vpandn   xmm10, xmm6, xmm10
       vpor     xmm9, xmm10, xmm9
       vmovd    r9d, xmm9
       vmovd    xmm9, r9d
       vpand    xmm9, xmm9, xmm0
       vpand    xmm10, xmm6, xmmword ptr [reloc @RWD112]
       vpand    xmm12, xmm7, xmmword ptr [reloc @RWD64]
       vpandn   xmm7, xmm7, xmmword ptr [reloc @RWD144]
       vpor     xmm7, xmm7, xmm12
       vmovd    r9d, xmm7
       vmovd    xmm7, r9d
       vpandn   xmm6, xmm6, xmm7
       vpor     xmm6, xmm6, xmm10
       vmovd    r9d, xmm6
       vmovd    xmm6, r9d
       vpandn   xmm6, xmm0, xmm6
 
G_M000_IG16:                ;; offset=0x0AF5
       vpor     xmm6, xmm6, xmm9
       vmovd    r9d, xmm6
       vmovd    xmm6, r10d
       vpand    xmm6, xmm6, xmm5
       vmovd    xmm7, r9d
       vpandn   xmm7, xmm5, xmm7
       vpor     xmm6, xmm7, xmm6
       vmovd    r10d, xmm6
       vsubss   xmm6, xmm8, xmm1
       vmovaps  xmm7, xmm6
       vandps   xmm7, xmm7, xmm5
       vxorps   xmm9, xmm9, xmm9
       vandps   xmm9, xmm9, xmm0
       vmovaps  xmm10, xmm1
       vandnps  xmm10, xmm0, xmm10
       vorps    xmm9, xmm10, xmm9
       vandnps  xmm9, xmm5, xmm9
       vorps    xmm7, xmm9, xmm7
       vmovss   dword ptr [rdx+0x20], xmm7
       vmovaps  xmm7, xmm1
       vandps   xmm7, xmm7, xmm5
       vmovaps  xmm9, xmm6
       vandps   xmm9, xmm9, xmm0
       vxorps   xmm10, xmm10, xmm10
       vandnps  xmm10, xmm0, xmm10
       vorps    xmm9, xmm10, xmm9
       vandnps  xmm9, xmm5, xmm9
       vorps    xmm7, xmm9, xmm7
       vmovss   dword ptr [rdx+0x50], xmm7
       vxorps   xmm7, xmm7, xmm7
       vandps   xmm7, xmm7, xmm5
       vandps   xmm1, xmm1, xmm0
       vandnps  xmm0, xmm0, xmm6
       vorps    xmm0, xmm0, xmm1
       vandnps  xmm0, xmm5, xmm0
       vorps    xmm0, xmm0, xmm7
       vmovss   dword ptr [rdx+0x80], xmm0
 
G_M000_IG17:                ;; offset=0x0B9C
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
       je       G_M000_IG20
 
G_M000_IG18:                ;; offset=0x0BC9
       vinsertps xmm2, xmm14, xmm14, 56
       vmovaps  xmm0, xmm11
       vinsertps xmm0, xmm0, xmm0, 56
       vmulps   xmm0, xmm0, xmm2
       vpermilps xmm1, xmm0, -11
       vaddps   xmm1, xmm1, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm1
       vmulss   xmm0, xmm0, xmm0
       vmulss   xmm1, xmm13, dword ptr [rsp+0xAC]
       vucomiss xmm1, xmm0
       jbe      SHORT G_M000_IG19
       mov      byte  ptr [rax], 1
 
G_M000_IG19:                ;; offset=0x0C08
       vmovaps  xmm2, xmm11
       mov      r10d, 7
       vmovss   xmm15, dword ptr [rsp+0x84]
       vmovss   dword ptr [rdx+0x20], xmm15
       vmovss   xmm0, dword ptr [rsp+0x88]
       vmovss   dword ptr [rdx+0x50], xmm0
       vmovss   xmm1, dword ptr [rsp+0x8C]
       vmovss   dword ptr [rdx+0x80], xmm1
       vmovss   dword ptr [rdx+0x90], xmm13
 
G_M000_IG20:                ;; offset=0x0C48
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
       jne      G_M000_IG22
 
G_M000_IG21:                ;; offset=0x0C7E
       vmovaps  xmm0, xmm2
       vmulps   xmm0, xmm0, xmmword ptr [reloc @RWD160]
       vaddps   xmm0, xmm0, xmm3
       vbroadcastss xmm1, dword ptr [rsp+0x1B0]
       vxorps   xmm3, xmm3, xmm3
       vcmpleps xmm1, xmm1, xmm3
       mov      eax, -1
       xor      edx, edx
       test     r8d, r8d
       cmove    eax, edx
       vmovd    xmm3, eax
       vpbroadcastd xmm3, xmm3
       vorps    xmm1, xmm3, xmm1
       vandps   xmm2, xmm2, xmm1
       vandnps  xmm0, xmm1, xmm0
       vorps    xmm2, xmm0, xmm2
       vmovaps  xmm0, xmm2
       vmovaps  xmm1, xmm2
       vmulss   xmm0, xmm0, xmm1
       vmovshdup xmm1, xmm2
       vmovshdup xmm3, xmm2
       vmulss   xmm1, xmm1, xmm3
       vaddss   xmm0, xmm0, xmm1
       vunpckhps xmm1, xmm2, xmm2
       vunpckhps xmm3, xmm2, xmm2
       vmulss   xmm1, xmm1, xmm3
       vaddss   xmm0, xmm0, xmm1
       vsqrtss  xmm0, xmm0, xmm0
       vdivss   xmm0, xmm8, xmm0
       vbroadcastss xmm0, xmm0
       vmovaps  xmm1, xmm2
       vmulps   xmm2, xmm0, xmm1
       vmovsd   qword ptr [rsp+0xB0], xmm2
       vextractps dword ptr [rsp+0xB8], xmm2, 2
 
G_M000_IG22:                ;; offset=0x0D1C
       vmovaps  xmm2, xmmword ptr [rsp+0xB0]
       vmovsd   qword ptr [rcx], xmm2
       vextractps dword ptr [rcx+0x08], xmm2, 2
       mov      rax, rcx
 
G_M000_IG23:                ;; offset=0x0D33
       vmovaps  xmm6, xmmword ptr [rsp+0x150]
       vmovaps  xmm7, xmmword ptr [rsp+0x140]
       vmovaps  xmm8, xmmword ptr [rsp+0x130]
       vmovaps  xmm9, xmmword ptr [rsp+0x120]
       vmovaps  xmm10, xmmword ptr [rsp+0x110]
       vmovaps  xmm11, xmmword ptr [rsp+0x100]
       vmovaps  xmm12, xmmword ptr [rsp+0xF0]
       vmovaps  xmm13, xmmword ptr [rsp+0xE0]
       vmovaps  xmm14, xmmword ptr [rsp+0xD0]
       vmovaps  xmm15, xmmword ptr [rsp+0xC0]
       add      rsp, 352
       pop      rbx
       pop      rsi
       pop      rdi
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

; Total bytes of code 3480

