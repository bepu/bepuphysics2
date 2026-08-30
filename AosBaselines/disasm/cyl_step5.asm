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

; Assembly listing for method AosBaselines.CylinderPairScalarTester:Test(byref,byref,float,byref,byref,byref,byref) (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rbp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 311 single block inlinees; 67 inlinees without PGO data

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
       vmovaps  xmm2, xmmword ptr [rbp-0x120]
       vmovaps  xmmword ptr [rbp-0x4F0], xmm2
       vinsertps xmm3, xmm2, xmm2, 56
       vmulps   xmm3, xmm3, xmm1
       vpermilps xmm4, xmm3, -11
       vaddps   xmm4, xmm4, xmm3
       vpermilps xmm3, xmm3, -86
       vaddps   xmm3, xmm3, xmm4
       vdivss   xmm4, xmm6, xmm3
       vmovss   xmm5, dword ptr [rbp-0x11C]
       vdivss   xmm11, xmm6, xmm5
       vmovss   dword ptr [rbp-0x144], xmm11
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
       vmovaps  xmmword ptr [rbp-0x500], xmm14
       vaddps   xmm13, xmm13, xmm14
       vbroadcastss xmm15, xmm5
       vxorps   xmm14, xmm14, xmm14
       vcmpltps xmm14, xmm15, xmm14
       vmovss   xmm15, dword ptr [rdi+0x04]
       vxorps   xmm11, xmm15, xmmword ptr [reloc @RWD16]
       vblendvps xmm11, xmm15, xmm11, xmm14
       vandps   xmm3, xmm3, xmmword ptr [reloc @RWD48]
       vmovss   dword ptr [rbp-0x480], xmm3
       vucomiss xmm3, dword ptr [reloc @RWD64]
       seta     al
       movzx    rax, al
       vandps   xmm5, xmm5, xmmword ptr [reloc @RWD48]
       vmovss   dword ptr [rbp-0x47C], xmm5
       vucomiss xmm5, dword ptr [reloc @RWD64]
       seta     cl
       movzx    rcx, cl
       vxorps   xmm14, xmm14, xmm14
       vxorps   xmm15, xmm15, xmm15
       vxorps   xmm5, xmm5, xmm5
       vmovaps  xmmword ptr [rbp-0x170], xmm5
       vxorps   xmm3, xmm3, xmm3
       vmovaps  xmmword ptr [rbp-0x180], xmm3
       vmovaps  xmm3, xmmword ptr [rbp-0x140]
       vmovups  xmmword ptr [rbp-0x54C], xmm3
       vmovss   xmm5, dword ptr [rbp-0x124]
       vxorps   xmm3, xmm5, xmmword ptr [reloc @RWD16]
       vbroadcastss xmm3, xmm3
       vmulps   xmm3, xmm3, xmm2
       vaddps   xmm3, xmm3, xmmword ptr [rbp-0x54C]
       vsubps   xmm3, xmm3, xmmword ptr [rbp-0x500]
       vmovaps  xmmword ptr [rbp-0x510], xmm3
       vinsertps xmm5, xmm3, xmm3, 56
       vmulps   xmm5, xmm5, xmm1
       vpermilps xmm3, xmm5, -11
 
G_M000_IG09:                ;; offset=0x0687
       vaddps   xmm3, xmm3, xmm5
       vpermilps xmm5, xmm5, -86
       vaddps   xmm3, xmm5, xmm3
       vbroadcastss xmm3, xmm3
       vmulps   xmm3, xmm3, xmm0
       vmovaps  xmm5, xmmword ptr [rbp-0x510]
       vsubps   xmm3, xmm5, xmm3
       vmovaps  xmmword ptr [rbp-0x190], xmm3
       vmovss   xmm5, dword ptr [rbp-0x140]
       vinsertps xmm5, xmm5, dword ptr [rbp-0x138], 28
       vxorps   xmm3, xmm0, xmmword ptr [reloc @RWD16]
       vandps   xmm3, xmm3, xmm12
       vandnps  xmm0, xmm12, xmm0
       vorps    xmm0, xmm0, xmm3
       vmovaps  xmmword ptr [rbp-0x1A0], xmm0
       vmovaps  xmm3, xmm0
       vmovsd   qword ptr [rbp-0x160], xmm13
       vextractps dword ptr [rbp-0x158], xmm13, 2
       vmovaps  xmm12, xmm13
       test     eax, ecx
       je       G_M000_IG22
       vmovss   xmm14, dword ptr [rbp-0x480]
       vmovss   dword ptr [rbp-0x1C4], xmm14
       vmovss   xmm0, dword ptr [rbp-0x47C]
       vmovss   dword ptr [rbp-0x1C8], xmm0
       vmovss   xmm14, dword ptr [reloc @RWD68]
       vmovss   dword ptr [rbp-0x524], xmm14
       vucomiss xmm14, xmm0
       seta     dl
       movzx    rdx, dl
       vmovaps  xmm0, xmm5
       vucomiss xmm14, dword ptr [rbp-0x1C4]
       seta     r8b
       movzx    r8, r8b
       test     r8d, edx
       jne      G_M000_IG21
       vmovaps  xmm0, xmmword ptr [rbp-0x120]
       vmovshdup xmm15, xmm12
       vsubss   xmm15, xmm15, xmm11
       vmulss   xmm15, xmm15, dword ptr [rbp-0x144]
       vmovss   dword ptr [rbp-0x2EC], xmm15
       vmovaps  xmm15, xmm0
       vmulss   xmm15, xmm15, dword ptr [rbp-0x2EC]
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovaps  xmm15, xmm12
       vsubss   xmm15, xmm15, dword ptr [rbp-0x538]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vunpckhps xmm0, xmm0, xmm0
       vmulss   xmm0, xmm0, dword ptr [rbp-0x2EC]
       vunpckhps xmm15, xmm12, xmm12
       vsubss   xmm0, xmm15, xmm0
       vmovups  xmm15, xmmword ptr [rbp-0x54C]
       vinsertps xmm0, xmm15, xmm0, 28
       vmovaps  xmm15, xmm0
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovaps  xmm15, xmm0
       vmulss   xmm15, xmm15, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovshdup xmm15, xmm0
       vmovss   dword ptr [rbp-0x53C], xmm15
       vmovshdup xmm15, xmm0
 
G_M000_IG10:                ;; offset=0x07F9
       vmulss   xmm15, xmm15, dword ptr [rbp-0x53C]
       vaddss   xmm15, xmm15, dword ptr [rbp-0x538]
       vsqrtss  xmm15, xmm15, xmm15
       vmovss   dword ptr [rbp-0x1D4], xmm15
       vdivss   xmm13, xmm6, xmm15
       vmovss   dword ptr [rbp-0x1D8], xmm13
       vbroadcastss xmm13, xmm13
       vmulps   xmm0, xmm13, xmm0
       vbroadcastss xmm13, xmm15
       vcmpltps xmm13, xmm13, xmmword ptr [reloc @RWD80]
       vandps   xmm14, xmm13, xmmword ptr [reloc @RWD96]
       vandnps  xmm0, xmm13, xmm0
       vorps    xmm0, xmm0, xmm14
       vmovaps  xmm13, xmm0
       vmovups  xmmword ptr [rbp-0x54C], xmm13
       vmovss   xmm14, dword ptr [rdi]
       vmovss   dword ptr [rbp-0x528], xmm14
       vbroadcastss xmm13, xmm14
       vmulps   xmm13, xmm13, xmmword ptr [rbp-0x54C]
       vmovaps  xmm15, xmm13
       vxorps   xmm15, xmm15, xmmword ptr [reloc @RWD16]
       vmovsd   qword ptr [rbp-0x1E0], xmm15
       vmovaps  xmm15, xmm13
       vinsertps xmm15, xmm15, xmm11, 16
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovshdup xmm15, xmm13
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x54C]
       vinsertps xmm15, xmm15, dword ptr [rbp-0x538], 40
       vmovaps  xmmword ptr [rbp-0x300], xmm15
       vmovaps  xmm14, xmm12
       vsubps   xmm15, xmm14, xmm15
       vinsertps xmm15, xmm15, xmm15, 56
       vmulps   xmm15, xmm15, xmm1
       vmovaps  xmmword ptr [rbp-0x330], xmm15
       vpermilps xmm15, xmm15, -11
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x330]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x330], -86
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vmulss   xmm15, xmm15, xmm4
       vbroadcastss xmm15, xmm15
       vmulps   xmm15, xmm15, xmm2
       vmovaps  xmmword ptr [rbp-0x310], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x300]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x310]
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vsubps   xmm15, xmm15, xmm14
       vinsertps xmm15, xmm15, xmm15, 56
       vmovaps  xmmword ptr [rbp-0x4C0], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0xE8]
       vinsertps xmm15, xmm15, xmm15, 56
       vmovaps  xmmword ptr [rbp-0x490], xmm15
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x4C0]
       vmovaps  xmmword ptr [rbp-0x340], xmm15
       vpermilps xmm15, xmm15, -11
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x340]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x340], -86
 
G_M000_IG11:                ;; offset=0x0991
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vmovss   dword ptr [rbp-0x314], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0xD0]
       vinsertps xmm15, xmm15, xmm15, 56
       vmovaps  xmmword ptr [rbp-0x4A0], xmm15
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x4C0]
       vmovaps  xmmword ptr [rbp-0x350], xmm15
       vmovss   xmm15, dword ptr [rbp-0x314]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x350], -11
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x350]
       vmovups  xmmword ptr [rbp-0x55C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x350], -86
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x55C]
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x54C]
       vinsertps xmm15, xmm15, dword ptr [rbp-0x538], 28
       vmovsd   qword ptr [rbp-0x1E8], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1E0]
       vinsertps xmm15, xmm15, xmm11, 16
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovsd   xmm15, qword ptr [rbp-0x1E0]
       vmovshdup xmm15, xmm15
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x54C]
       vinsertps xmm15, xmm15, dword ptr [rbp-0x538], 40
       vmovaps  xmmword ptr [rbp-0x360], xmm15
       vsubps   xmm15, xmm14, xmm15
       vinsertps xmm15, xmm15, xmm15, 56
       vmulps   xmm15, xmm15, xmm1
       vmovaps  xmmword ptr [rbp-0x390], xmm15
       vpermilps xmm15, xmm15, -11
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x390]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x390], -86
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vmulss   xmm15, xmm15, xmm4
       vbroadcastss xmm15, xmm15
       vmulps   xmm15, xmm15, xmm2
       vmovaps  xmmword ptr [rbp-0x370], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x360]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x370]
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vsubps   xmm15, xmm15, xmm14
       vinsertps xmm15, xmm15, xmm15, 56
       vmovaps  xmmword ptr [rbp-0x4D0], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x490]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x4D0]
       vmovaps  xmmword ptr [rbp-0x3A0], xmm15
       vpermilps xmm15, xmm15, -11
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x3A0]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x3A0], -86
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vmovss   dword ptr [rbp-0x374], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x4A0]
 
G_M000_IG12:                ;; offset=0x0B41
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x4D0]
       vmovaps  xmmword ptr [rbp-0x3B0], xmm15
       vmovss   xmm15, dword ptr [rbp-0x374]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x3B0], -11
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x3B0]
       vmovups  xmmword ptr [rbp-0x55C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x3B0], -86
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x55C]
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x54C]
       vinsertps xmm15, xmm15, dword ptr [rbp-0x538], 28
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovsd   xmm15, qword ptr [rbp-0x1E8]
       vmovups  xmmword ptr [rbp-0x55C], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x54C]
       vsubps   xmm15, xmm15, xmmword ptr [rbp-0x55C]
       vmovsd   qword ptr [rbp-0x1F0], xmm15
       vmovsd   xmm15, qword ptr [rbp-0x1E0]
       vmovups  xmmword ptr [rbp-0x55C], xmm15
       vmovaps  xmm15, xmm13
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x55C]
       vsubps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vmovsd   qword ptr [rbp-0x2C8], xmm15
       vmovss   xmm15, dword ptr [rsi]
       vmovss   dword ptr [rbp-0x3C4], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1F0]
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1F0]
       vmulss   xmm15, xmm15, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovsd   xmm15, qword ptr [rbp-0x1F0]
       vmovshdup xmm15, xmm15
       vmovss   dword ptr [rbp-0x52C], xmm15
       vmulss   xmm15, xmm15, xmm15
       vaddss   xmm15, xmm15, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x3B4], xmm15
       vdivss   xmm15, xmm6, xmm15
       vmovss   dword ptr [rbp-0x3B8], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1E8]
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1F0]
       vmulss   xmm15, xmm15, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovsd   xmm15, qword ptr [rbp-0x1E8]
       vmovshdup xmm15, xmm15
       vmovss   dword ptr [rbp-0x530], xmm15
       vmovss   xmm2, dword ptr [rbp-0x52C]
       vmulss   xmm2, xmm15, xmm2
       vaddss   xmm2, xmm2, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x3BC], xmm2
       vmovss   xmm2, dword ptr [rbp-0x1E8]
       vmovss   xmm15, dword ptr [rbp-0x1E8]
       vmulss   xmm2, xmm2, xmm15
       vmovss   xmm15, dword ptr [rbp-0x530]
       vmulss   xmm15, xmm15, xmm15
       vaddss   xmm2, xmm2, xmm15
       vmovss   xmm15, dword ptr [rbp-0x3C4]
 
G_M000_IG13:                ;; offset=0x0CFF
       vmulss   xmm15, xmm15, xmm15
       vmovss   dword ptr [rbp-0x4A4], xmm15
       vsubss   xmm2, xmm2, xmm15
       vmovss   dword ptr [rbp-0x3C0], xmm2
       vmovss   xmm2, dword ptr [rbp-0x3BC]
       vmulss   xmm2, xmm2, xmm2
       vmovss   xmm15, dword ptr [rbp-0x3B4]
       vmulss   xmm15, xmm15, dword ptr [rbp-0x3C0]
       vsubss   xmm2, xmm2, xmm15
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm2, xmm15, xmm2
       vsqrtss  xmm2, xmm2, xmm2
       vmulss   xmm2, xmm2, dword ptr [rbp-0x3B8]
       vmovss   dword ptr [rbp-0x3C8], xmm2
       vmovss   xmm2, dword ptr [rbp-0x3BC]
       vxorps   xmm2, xmm2, xmmword ptr [reloc @RWD16]
       vmulss   xmm2, xmm2, dword ptr [rbp-0x3B8]
       vmovss   dword ptr [rbp-0x3CC], xmm2
       vsubss   xmm15, xmm2, dword ptr [rbp-0x3C8]
       vmovss   dword ptr [rbp-0x1F4], xmm15
       vaddss   xmm15, xmm2, dword ptr [rbp-0x3C8]
       vmovss   dword ptr [rbp-0x1F8], xmm15
       vmovss   xmm2, dword ptr [rbp-0x3B4]
       vandps   xmm2, xmm2, xmmword ptr [reloc @RWD48]
       vbroadcastss xmm2, xmm2
       vcmpltps xmm2, xmm2, xmmword ptr [reloc @RWD112]
       vmovss   xmm15, dword ptr [rbp-0x3CC]
       vmovaps  xmmword ptr [rbp-0x520], xmm15
       vandps   xmm15, xmm15, xmm2
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1F4]
       vandnps  xmm15, xmm2, xmm15
       vorps    xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vmovss   dword ptr [rbp-0x1F4], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x520]
       vandps   xmm15, xmm15, xmm2
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1F8]
       vandnps  xmm2, xmm2, xmm15
       vorps    xmm15, xmm2, xmmword ptr [rbp-0x54C]
       vmovss   dword ptr [rbp-0x1F8], xmm15
       vmovss   xmm2, dword ptr [rbp-0x1F4]
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm2, xmm2, xmm15
       vmovss   dword ptr [rbp-0x1FC], xmm2
       vmovss   xmm15, dword ptr [rbp-0x1F8]
       vminss   xmm15, xmm15, dword ptr [reloc @RWD00]
       vmovss   dword ptr [rbp-0x200], xmm15
       vmovsd   xmm15, qword ptr [rbp-0x2C8]
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm15
       vmovaps  xmm15, xmm13
       vaddps   xmm2, xmm2, xmm15
       vmovsd   qword ptr [rbp-0x1D0], xmm2
       vmovsd   xmm15, qword ptr [rbp-0x2C8]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vbroadcastss xmm15, dword ptr [rbp-0x200]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vaddps   xmm13, xmm15, xmm13
       vmovsd   qword ptr [rbp-0x208], xmm13
 
G_M000_IG14:                ;; offset=0x0EA3
       vmovss   xmm15, dword ptr [rbp-0x528]
       vmulss   xmm15, xmm15, xmm15
       vsubss   xmm15, xmm15, dword ptr [rbp-0x4A4]
       vmulss   xmm15, xmm15, dword ptr [rbp-0x1D8]
       vaddss   xmm15, xmm15, dword ptr [rbp-0x1D4]
       vmulss   xmm15, xmm15, dword ptr [reloc @RWD128]
       vmovss   dword ptr [rbp-0x20C], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1D4]
       vxorps   xmm13, xmm13, xmm13
       vmaxss   xmm13, xmm13, dword ptr [rbp-0x20C]
       vminss   xmm13, xmm15, xmm13
       vbroadcastss xmm13, xmm13
       vmovaps  xmm15, xmm0
       vmulps   xmm13, xmm13, xmm15
       vmovshdup xmm15, xmm0
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovaps  xmm15, xmm0
       vxorps   xmm2, xmm15, xmmword ptr [reloc @RWD16]
       vmovups  xmm15, xmmword ptr [rbp-0x54C]
       vinsertps xmm2, xmm15, xmm2, 28
       vmovaps  xmm15, xmm13
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovaps  xmm15, xmm2
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vmovsd   qword ptr [rbp-0x218], xmm15
       vmovaps  xmm15, xmm13
       vinsertps xmm15, xmm15, xmm11, 16
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovshdup xmm15, xmm13
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x54C]
       vinsertps xmm15, xmm15, dword ptr [rbp-0x538], 40
       vmovaps  xmmword ptr [rbp-0x3E0], xmm15
       vsubps   xmm15, xmm14, xmm15
       vinsertps xmm15, xmm15, xmm15, 56
       vmulps   xmm15, xmm15, xmm1
       vmovaps  xmmword ptr [rbp-0x410], xmm15
       vpermilps xmm15, xmm15, -11
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x410]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x410], -86
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vmulss   xmm15, xmm15, xmm4
       vbroadcastss xmm15, xmm15
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x4F0]
       vmovaps  xmmword ptr [rbp-0x3F0], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x3E0]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x3F0]
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vsubps   xmm15, xmm15, xmm14
       vinsertps xmm15, xmm15, xmm15, 56
       vmovaps  xmmword ptr [rbp-0x4E0], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x490]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x4E0]
       vmovaps  xmmword ptr [rbp-0x420], xmm15
       vpermilps xmm15, xmm15, -11
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x420]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x420], -86
 
G_M000_IG15:                ;; offset=0x1044
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vmovss   dword ptr [rbp-0x3F4], xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x4A0]
       vmulps   xmm15, xmm15, xmmword ptr [rbp-0x4E0]
       vmovaps  xmmword ptr [rbp-0x430], xmm15
       vmovss   xmm15, dword ptr [rbp-0x3F4]
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x430], -11
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x430]
       vmovups  xmmword ptr [rbp-0x55C], xmm15
       vpermilps xmm15, xmmword ptr [rbp-0x430], -86
       vaddps   xmm15, xmm15, xmmword ptr [rbp-0x55C]
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x54C]
       vinsertps xmm15, xmm15, dword ptr [rbp-0x538], 28
       vmovsd   qword ptr [rbp-0x220], xmm15
       vmovss   xmm15, dword ptr [rbp-0x218]
       vinsertps xmm15, xmm15, xmm11, 16
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovsd   xmm15, qword ptr [rbp-0x218]
       vmovshdup xmm15, xmm15
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovups  xmm15, xmmword ptr [rbp-0x54C]
       vinsertps xmm15, xmm15, dword ptr [rbp-0x538], 40
       vmovaps  xmmword ptr [rbp-0x440], xmm15
       vsubps   xmm15, xmm14, xmm15
       vinsertps xmm15, xmm15, xmm15, 56
       vmulps   xmm1, xmm15, xmm1
       vpermilps xmm15, xmm1, -11
       vaddps   xmm15, xmm15, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm15
       vmulss   xmm1, xmm1, xmm4
       vbroadcastss xmm1, xmm1
       vmulps   xmm1, xmm1, xmmword ptr [rbp-0x4F0]
       vmovaps  xmm15, xmmword ptr [rbp-0x440]
       vaddps   xmm1, xmm1, xmm15
       vsubps   xmm1, xmm1, xmm14
       vinsertps xmm1, xmm1, xmm1, 56
       vmovaps  xmm15, xmmword ptr [rbp-0x490]
       vmulps   xmm14, xmm15, xmm1
       vpermilps xmm15, xmm14, -11
       vaddps   xmm15, xmm15, xmm14
       vpermilps xmm14, xmm14, -86
       vaddps   xmm14, xmm14, xmm15
       vmovaps  xmm15, xmmword ptr [rbp-0x4A0]
       vmulps   xmm1, xmm15, xmm1
       vpermilps xmm15, xmm1, -11
       vaddps   xmm15, xmm15, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm15
       vinsertps xmm1, xmm14, xmm1, 28
       vmovsd   xmm15, qword ptr [rbp-0x220]
       vmovaps  xmm14, xmm15
       vsubps   xmm1, xmm1, xmm14
       vmovaps  xmm14, xmm1
       vmovaps  xmm15, xmm1
       vmulss   xmm14, xmm14, xmm15
 
G_M000_IG16:                ;; offset=0x11C4
       vmovshdup xmm15, xmm1
       vmovss   dword ptr [rbp-0x534], xmm15
       vmulss   xmm15, xmm15, xmm15
       vaddss   xmm14, xmm14, xmm15
       vmovss   dword ptr [rbp-0x444], xmm14
       vdivss   xmm15, xmm6, xmm14
       vmovss   dword ptr [rbp-0x448], xmm15
       vmovss   xmm15, dword ptr [rbp-0x220]
       vmulss   xmm1, xmm15, xmm1
       vmovsd   xmm15, qword ptr [rbp-0x220]
       vmovshdup xmm15, xmm15
       vmovss   xmm14, dword ptr [rbp-0x534]
       vmulss   xmm14, xmm15, xmm14
       vaddss   xmm1, xmm1, xmm14
       vmovss   dword ptr [rbp-0x44C], xmm1
       vmovss   xmm14, dword ptr [rbp-0x220]
       vmovss   xmm1, dword ptr [rbp-0x220]
       vmulss   xmm1, xmm14, xmm1
       vmulss   xmm14, xmm15, xmm15
       vaddss   xmm1, xmm1, xmm14
       vsubss   xmm1, xmm1, dword ptr [rbp-0x4A4]
       vmovss   xmm14, dword ptr [rbp-0x44C]
       vmulss   xmm15, xmm14, xmm14
       vmovss   xmm14, dword ptr [rbp-0x444]
       vmulss   xmm1, xmm14, xmm1
       vsubss   xmm1, xmm15, xmm1
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm1, xmm15, xmm1
       vsqrtss  xmm1, xmm1, xmm1
       vmulss   xmm1, xmm1, dword ptr [rbp-0x448]
       vmovss   xmm14, dword ptr [rbp-0x44C]
       vxorps   xmm14, xmm14, xmmword ptr [reloc @RWD16]
       vmulss   xmm14, xmm14, dword ptr [rbp-0x448]
       vsubss   xmm15, xmm14, xmm1
       vmovss   dword ptr [rbp-0x224], xmm15
       vaddss   xmm1, xmm14, xmm1
       vmovss   dword ptr [rbp-0x228], xmm1
       vmovss   xmm15, dword ptr [rbp-0x444]
       vandps   xmm15, xmm15, xmmword ptr [reloc @RWD48]
       vbroadcastss xmm1, xmm15
       vcmpltps xmm1, xmm1, xmmword ptr [reloc @RWD112]
       vmovaps  xmm15, xmm14
       vandps   xmm15, xmm15, xmm1
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovss   xmm15, dword ptr [rbp-0x224]
       vandnps  xmm15, xmm1, xmm15
       vorps    xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vmovss   dword ptr [rbp-0x224], xmm15
       vandps   xmm14, xmm14, xmm1
       vmovss   xmm15, dword ptr [rbp-0x228]
       vandnps  xmm1, xmm1, xmm15
       vorps    xmm1, xmm1, xmm14
       vmovss   dword ptr [rbp-0x228], xmm1
       vmovss   xmm15, dword ptr [rbp-0x528]
       vmovaps  xmm14, xmm2
       vmovaps  xmm1, xmm2
       vmulss   xmm1, xmm14, xmm1
       vmovss   dword ptr [rbp-0x538], xmm1
       vmovshdup xmm14, xmm2
       vmovshdup xmm1, xmm2
       vmulss   xmm1, xmm14, xmm1
 
G_M000_IG17:                ;; offset=0x133C
       vaddss   xmm1, xmm1, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x450], xmm1
       vdivss   xmm14, xmm6, xmm1
       vmovss   dword ptr [rbp-0x454], xmm14
       vmovaps  xmm14, xmm13
       vmovaps  xmm1, xmm2
       vmulss   xmm1, xmm14, xmm1
       vmovss   dword ptr [rbp-0x538], xmm1
       vmovshdup xmm14, xmm13
       vmovshdup xmm1, xmm2
       vmulss   xmm1, xmm14, xmm1
       vaddss   xmm1, xmm1, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x458], xmm1
       vmovaps  xmm14, xmm13
       vmovaps  xmm1, xmm13
       vmulss   xmm1, xmm14, xmm1
       vmovss   dword ptr [rbp-0x538], xmm1
       vmovshdup xmm14, xmm13
       vmovshdup xmm1, xmm13
       vmulss   xmm1, xmm14, xmm1
       vaddss   xmm1, xmm1, dword ptr [rbp-0x538]
       vmulss   xmm14, xmm15, xmm15
       vsubss   xmm1, xmm1, xmm14
       vmovss   xmm14, dword ptr [rbp-0x458]
       vmulss   xmm15, xmm14, xmm14
       vmovss   xmm14, dword ptr [rbp-0x450]
       vmulss   xmm1, xmm14, xmm1
       vsubss   xmm1, xmm15, xmm1
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm1, xmm15, xmm1
       vsqrtss  xmm1, xmm1, xmm1
       vmulss   xmm1, xmm1, dword ptr [rbp-0x454]
       vmovss   xmm14, dword ptr [rbp-0x458]
       vxorps   xmm14, xmm14, xmmword ptr [reloc @RWD16]
       vmulss   xmm14, xmm14, dword ptr [rbp-0x454]
       vsubss   xmm15, xmm14, xmm1
       vmovss   dword ptr [rbp-0x22C], xmm15
       vaddss   xmm1, xmm14, xmm1
       vmovss   dword ptr [rbp-0x230], xmm1
       vmovss   xmm15, dword ptr [rbp-0x450]
       vandps   xmm15, xmm15, xmmword ptr [reloc @RWD48]
       vbroadcastss xmm1, xmm15
       vcmpltps xmm1, xmm1, xmmword ptr [reloc @RWD112]
       vmovaps  xmm15, xmm14
       vandps   xmm15, xmm15, xmm1
       vmovups  xmmword ptr [rbp-0x54C], xmm15
       vmovss   xmm15, dword ptr [rbp-0x22C]
       vandnps  xmm15, xmm1, xmm15
       vorps    xmm15, xmm15, xmmword ptr [rbp-0x54C]
       vmovss   dword ptr [rbp-0x22C], xmm15
       vandps   xmm14, xmm14, xmm1
       vmovss   xmm15, dword ptr [rbp-0x230]
       vandnps  xmm1, xmm1, xmm15
       vorps    xmm1, xmm1, xmm14
       vmovss   xmm14, dword ptr [rbp-0x224]
       vmaxss   xmm14, xmm14, dword ptr [rbp-0x22C]
       vmovss   dword ptr [rbp-0x234], xmm14
       vmovss   xmm15, dword ptr [rbp-0x228]
       vminss   xmm1, xmm15, xmm1
       vmovss   dword ptr [rbp-0x238], xmm1
       vmovaps  xmm15, xmm2
       vbroadcastss xmm14, xmm14
       vmulps   xmm14, xmm14, xmm15
 
G_M000_IG18:                ;; offset=0x14BE
       vmovaps  xmm15, xmm2
       vbroadcastss xmm1, xmm1
       vmulps   xmm1, xmm1, xmm15
       vmovaps  xmm15, xmm13
       vaddps   xmm14, xmm14, xmm15
       vmovaps  xmm15, xmm13
       vaddps   xmm1, xmm1, xmm15
       vmovsd   qword ptr [rbp-0x240], xmm1
       vmovss   xmm15, dword ptr [rbp-0x1C4]
       vsubss   xmm15, xmm15, dword ptr [rbp-0x524]
       vmulss   xmm15, xmm15, dword ptr [reloc @RWD132]
       vmovss   dword ptr [rbp-0x538], xmm15
       vbroadcastss xmm15, dword ptr [reloc @RWD00]
       vminss   xmm15, xmm15, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x538], xmm15
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm15, xmm15, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x538], xmm15
       vmovss   xmm15, dword ptr [rbp-0x1C8]
       vsubss   xmm15, xmm15, dword ptr [rbp-0x524]
       vmulss   xmm15, xmm15, dword ptr [reloc @RWD132]
       vmovss   dword ptr [rbp-0x53C], xmm15
       vbroadcastss xmm15, dword ptr [reloc @RWD00]
       vminss   xmm15, xmm15, dword ptr [rbp-0x53C]
       vmovss   dword ptr [rbp-0x53C], xmm15
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm15, xmm15, dword ptr [rbp-0x53C]
       vmulss   xmm15, xmm15, dword ptr [rbp-0x538]
       vsubss   xmm1, xmm6, xmm15
       vmovss   dword ptr [rbp-0x244], xmm1
       vmovaps  xmm1, xmm5
       vsubps   xmm1, xmm1, xmm13
       vmovaps  xmm13, xmm2
       vmovss   dword ptr [rbp-0x538], xmm13
       vmovaps  xmm13, xmm1
       vmulss   xmm13, xmm13, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x538], xmm13
       vmovshdup xmm2, xmm2
       vmovshdup xmm13, xmm1
       vmulss   xmm2, xmm2, xmm13
       vaddss   xmm2, xmm2, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x248], xmm2
       vmovaps  xmm13, xmm0
       vmovaps  xmm2, xmm1
       vmulss   xmm2, xmm13, xmm2
       vmovshdup xmm0, xmm0
       vmovshdup xmm1, xmm1
       vmulss   xmm0, xmm0, xmm1
       vaddss   xmm0, xmm2, xmm0
       vandps   xmm1, xmm0, xmmword ptr [reloc @RWD48]
       vbroadcastss xmm1, xmm1
       vmovss   xmm13, dword ptr [rbp-0x248]
       vandps   xmm2, xmm13, xmmword ptr [reloc @RWD48]
       vbroadcastss xmm2, xmm2
       vcmpgtps xmm1, xmm1, xmm2
       vbroadcastss xmm2, xmm0
       vxorps   xmm13, xmm13, xmm13
       vcmpgtps xmm2, xmm2, xmm13
       vandps   xmm2, xmm2, xmm1
       vmovaps  xmmword ptr [rbp-0x260], xmm2
       vbroadcastss xmm0, xmm0
       vcmpleps xmm0, xmm0, xmm13
       vmovaps  xmmword ptr [rbp-0x2E0], xmm0
       vbroadcastss xmm13, dword ptr [rbp-0x248]
 
G_M000_IG19:                ;; offset=0x1652
       vxorps   xmm2, xmm2, xmm2
       vcmpltps xmm2, xmm13, xmm2
       vandnps  xmm2, xmm1, xmm2
       vmovaps  xmmword ptr [rbp-0x270], xmm2
       vbroadcastss xmm13, dword ptr [rbp-0x248]
       vxorps   xmm2, xmm2, xmm2
       vcmpgeps xmm2, xmm13, xmm2
       vandnps  xmm2, xmm1, xmm2
       vmovaps  xmmword ptr [rbp-0x280], xmm2
       vmovaps  xmm13, xmm5
       vbroadcastss xmm2, dword ptr [rbp-0x244]
       vmulps   xmm2, xmm2, xmm13
       vmovsd   xmm13, qword ptr [rbp-0x1D0]
       vmovaps  xmm0, xmm13
       vmovups  xmmword ptr [rbp-0x54C], xmm0
       vbroadcastss xmm0, xmm15
       vmulps   xmm0, xmm0, xmmword ptr [rbp-0x54C]
       vmovsd   qword ptr [rbp-0x2E8], xmm0
       vandps   xmm1, xmm1, xmmword ptr [rbp-0x2E0]
       vmovaps  xmm0, xmm2
       vmovups  xmmword ptr [rbp-0x54C], xmm0
       vmovsd   xmm0, qword ptr [rbp-0x2E8]
       vaddps   xmm0, xmm0, xmmword ptr [rbp-0x54C]
       vandps   xmm0, xmm0, xmmword ptr [rbp-0x260]
       vmovups  xmmword ptr [rbp-0x54C], xmm0
       vmovaps  xmm0, xmmword ptr [rbp-0x260]
       vandnps  xmm0, xmm0, xmm13
       vorps    xmm13, xmm0, xmmword ptr [rbp-0x54C]
       vmovsd   xmm0, qword ptr [rbp-0x208]
       vmovups  xmmword ptr [rbp-0x54C], xmm0
       vbroadcastss xmm0, xmm15
       vmulps   xmm0, xmm0, xmmword ptr [rbp-0x54C]
       vmovups  xmmword ptr [rbp-0x54C], xmm0
       vmovaps  xmm0, xmm2
       vmovups  xmmword ptr [rbp-0x55C], xmm0
       vmovups  xmm0, xmmword ptr [rbp-0x54C]
       vaddps   xmm0, xmm0, xmmword ptr [rbp-0x55C]
       vandps   xmm0, xmm0, xmm1
       vmovups  xmmword ptr [rbp-0x55C], xmm0
       vmovsd   xmm0, qword ptr [rbp-0x208]
       vandnps  xmm0, xmm1, xmm0
       vorps    xmm0, xmm0, xmmword ptr [rbp-0x55C]
       vmovsd   qword ptr [rbp-0x460], xmm0
       vmovaps  xmm1, xmm14
       vbroadcastss xmm0, xmm15
       vmulps   xmm0, xmm0, xmm1
       vmovaps  xmm1, xmm2
       vaddps   xmm0, xmm0, xmm1
       vmovaps  xmm1, xmmword ptr [rbp-0x270]
       vandps   xmm0, xmm0, xmm1
       vandnps  xmm1, xmm1, xmm14
       vorps    xmm0, xmm1, xmm0
       vmovsd   xmm1, qword ptr [rbp-0x240]
       vmovaps  xmm14, xmm1
       vbroadcastss xmm15, xmm15
       vmulps   xmm14, xmm15, xmm14
       vaddps   xmm2, xmm14, xmm2
       vmovaps  xmm14, xmmword ptr [rbp-0x280]
       vandps   xmm2, xmm2, xmm14
       vandnps  xmm1, xmm14, xmm1
       vorps    xmm1, xmm1, xmm2
       vmovsd   xmm2, qword ptr [rbp-0x460]
       vmovaps  xmm14, xmm2
 
G_M000_IG20:                ;; offset=0x17D9
       vinsertps xmm14, xmm14, xmm11, 16
       vmovshdup xmm2, xmm2
       vinsertps xmm15, xmm14, xmm2, 40
       vmovaps  xmm2, xmm0
       vinsertps xmm2, xmm2, xmm11, 16
       vmovshdup xmm0, xmm0
       vinsertps xmm0, xmm2, xmm0, 40
       vmovaps  xmm2, xmm1
       vinsertps xmm2, xmm2, xmm11, 16
       vmovshdup xmm1, xmm1
       vinsertps xmm1, xmm2, xmm1, 40
       vmovaps  xmmword ptr [rbp-0x180], xmm1
       vmovss   xmm2, dword ptr [rbp-0x200]
       vucomiss xmm2, dword ptr [rbp-0x1FC]
       seta     dl
       mov      byte  ptr [rbx+0x5D], dl
       movzx    rdx, byte  ptr [rbx+0x5D]
       mov      byte  ptr [rbx+0x5E], dl
       movzx    rdx, byte  ptr [rbx+0x5D]
       vmovss   xmm2, dword ptr [rbp-0x238]
       vucomiss xmm2, dword ptr [rbp-0x234]
       seta     r8b
       movzx    r8, r8b
       and      edx, r8d
       mov      byte  ptr [rbx+0x5F], dl
       vmovsd   qword ptr [rbp-0x170], xmm0
       vextractps dword ptr [rbp-0x168], xmm0, 2
       vmovaps  xmm0, xmm13
 
G_M000_IG21:                ;; offset=0x186F
       vmovaps  xmm2, xmm0
       vinsertps xmm2, xmm2, xmm11, 16
       vmovshdup xmm0, xmm0
       vinsertps xmm14, xmm2, xmm0, 40
       mov      byte  ptr [rbx+0x5C], 1
 
G_M000_IG22:                ;; offset=0x1887
       vmovups  xmm2, xmmword ptr [rbp-0xE8]
       vinsertps xmm2, xmm2, xmm2, 56
       vmovaps  xmm0, xmmword ptr [rbp-0x120]
       vinsertps xmm0, xmm0, xmm0, 56
       vmulps   xmm0, xmm0, xmm2
       vpermilps xmm2, xmm0, -11
       vaddps   xmm2, xmm2, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm2
       vmovaps  xmm2, xmmword ptr [rbp-0xD0]
       vinsertps xmm2, xmm2, xmm2, 56
       vmovaps  xmm1, xmmword ptr [rbp-0x120]
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm1, xmm1, xmm2
       vpermilps xmm2, xmm1, -11
       vaddps   xmm2, xmm2, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm2
       vmulss   xmm2, xmm0, xmm0
       vmulss   xmm13, xmm1, xmm1
       vaddss   xmm2, xmm2, xmm13
       vsqrtss  xmm2, xmm2, xmm2
       vdivss   xmm2, xmm6, xmm2
       vmulss   xmm0, xmm0, xmm2
       vbroadcastss xmm0, xmm0
       vmovups  xmm13, xmmword ptr [rbp-0xE8]
       vmulps   xmm0, xmm0, xmm13
       vmulss   xmm1, xmm1, xmm2
       vbroadcastss xmm1, xmm1
       vmovaps  xmm2, xmmword ptr [rbp-0xD0]
       vmulps   xmm1, xmm1, xmm2
       vaddps   xmm0, xmm1, xmm0
       vmovaps  xmmword ptr [rbp-0x1B0], xmm0
       vmovaps  xmm1, xmmword ptr [rbp-0x190]
       vmovaps  xmm2, xmmword ptr [rbp-0x110]
       vaddps   xmm1, xmm2, xmm1
       vmovaps  xmmword ptr [rbp-0x1C0], xmm1
       vmovaps  xmm2, xmm5
       vmovshdup xmm13, xmm5
       vinsertps xmm2, xmm2, xmm13, 42
       cmp      eax, ecx
       je       G_M000_IG27
       vmovaps  xmm3, xmm1
       vmovups  xmm14, xmmword ptr [rbp-0xDC]
       vaddps   xmm3, xmm14, xmm3
       vmovaps  xmm15, xmm2
       vinsertps xmm12, xmm15, xmm6, 16
       vunpckhps xmm13, xmm2, xmm2
       vinsertps xmm12, xmm12, xmm13, 40
       vmovaps  xmmword ptr [rbp-0x290], xmm12
       vmovaps  xmm13, xmmword ptr [rbp-0x120]
       vmovaps  xmm14, xmmword ptr [rbp-0x160]
       vmovaps  xmm15, xmm2
       vsubps   xmm15, xmm14, xmm15
       vinsertps xmm15, xmm15, xmm15, 56
       vmovups  xmm0, xmmword ptr [rbp-0xDC]
       vinsertps xmm0, xmm0, xmm0, 56
 
G_M000_IG23:                ;; offset=0x19C6
       vmulps   xmm0, xmm0, xmm15
       vpermilps xmm15, xmm0, -11
       vaddps   xmm15, xmm15, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm15
       vmulss   xmm0, xmm0, xmm4
       vbroadcastss xmm0, xmm0
       vmulps   xmm0, xmm0, xmm13
       vmovaps  xmm13, xmm2
       vaddps   xmm0, xmm0, xmm13
       vsubps   xmm0, xmm0, xmm14
       vmovaps  xmm13, xmm0
       vinsertps xmm13, xmm13, xmm13, 56
       vmovups  xmm15, xmmword ptr [rbp-0xE8]
       vinsertps xmm15, xmm15, xmm15, 56
       vmulps   xmm13, xmm15, xmm13
       vpermilps xmm15, xmm13, -11
       vaddps   xmm15, xmm15, xmm13
       vpermilps xmm13, xmm13, -86
       vaddps   xmm13, xmm13, xmm15
       vinsertps xmm0, xmm0, xmm0, 56
       vmovaps  xmm15, xmmword ptr [rbp-0xD0]
       vinsertps xmm15, xmm15, xmm15, 56
       vmulps   xmm0, xmm15, xmm0
       vpermilps xmm15, xmm0, -11
       vaddps   xmm15, xmm15, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm15
       vinsertps xmm0, xmm13, xmm0, 28
       vmovsd   qword ptr [rbp-0x298], xmm0
       vmovaps  xmm13, xmmword ptr [rbp-0x120]
       vmovshdup xmm15, xmm1
       vsubss   xmm15, xmm15, xmm11
       vmulss   xmm15, xmm15, dword ptr [rbp-0x144]
       vmovaps  xmm0, xmm13
       vmulss   xmm0, xmm0, xmm15
       vmovaps  xmm12, xmm1
       vsubss   xmm0, xmm12, xmm0
       vunpckhps xmm12, xmm13, xmm13
       vmulss   xmm12, xmm12, xmm15
       vunpckhps xmm13, xmm1, xmm1
       vsubss   xmm12, xmm13, xmm12
       vinsertps xmm0, xmm0, xmm12, 28
       vmovaps  xmm12, xmmword ptr [rbp-0x120]
       vmovaps  xmm13, xmmword ptr [rbp-0x290]
       vmovaps  xmm15, xmm13
       vsubps   xmm15, xmm14, xmm15
       vinsertps xmm15, xmm15, xmm15, 56
       vmovups  xmm13, xmmword ptr [rbp-0xDC]
       vinsertps xmm13, xmm13, xmm13, 56
       vmulps   xmm13, xmm13, xmm15
       vpermilps xmm15, xmm13, -11
       vaddps   xmm15, xmm15, xmm13
       vpermilps xmm13, xmm13, -86
       vaddps   xmm13, xmm13, xmm15
       vmulss   xmm4, xmm13, xmm4
       vbroadcastss xmm4, xmm4
       vmulps   xmm4, xmm4, xmm12
       vmovaps  xmm13, xmmword ptr [rbp-0x290]
 
G_M000_IG24:                ;; offset=0x1B0D
       vaddps   xmm4, xmm4, xmm13
       vsubps   xmm4, xmm4, xmm14
       vmovaps  xmm12, xmm4
       vinsertps xmm12, xmm12, xmm12, 56
       vmovups  xmm13, xmmword ptr [rbp-0xE8]
       vinsertps xmm13, xmm13, xmm13, 56
       vmulps   xmm12, xmm13, xmm12
       vpermilps xmm13, xmm12, -11
       vaddps   xmm13, xmm13, xmm12
       vpermilps xmm12, xmm12, -86
       vaddps   xmm12, xmm12, xmm13
       vinsertps xmm4, xmm4, xmm4, 56
       vmovaps  xmm13, xmmword ptr [rbp-0xD0]
       vinsertps xmm13, xmm13, xmm13, 56
       vmulps   xmm4, xmm13, xmm4
       vpermilps xmm13, xmm4, -11
       vaddps   xmm13, xmm13, xmm4
       vpermilps xmm4, xmm4, -86
       vaddps   xmm4, xmm4, xmm13
       vinsertps xmm4, xmm12, xmm4, 28
       vmovsd   qword ptr [rbp-0x2A0], xmm4
       vmovaps  xmm12, xmmword ptr [rbp-0x120]
       vmovshdup xmm13, xmm3
       vsubss   xmm13, xmm13, xmm11
       vmulss   xmm13, xmm13, dword ptr [rbp-0x144]
       vmovaps  xmm15, xmm12
       vmulss   xmm15, xmm15, xmm13
       vmovaps  xmm4, xmm3
       vsubss   xmm4, xmm4, xmm15
       vunpckhps xmm12, xmm12, xmm12
       vmulss   xmm12, xmm12, xmm13
       vunpckhps xmm3, xmm3, xmm3
       vsubss   xmm3, xmm3, xmm12
       vinsertps xmm3, xmm4, xmm3, 28
       mov      edx, -1
       xor      r8d, r8d
       test     eax, eax
       cmove    edx, r8d
       vmovd    xmm4, edx
       vpbroadcastd xmm4, xmm4
       vmovsd   xmm12, qword ptr [rbp-0x298]
       vandps   xmm12, xmm12, xmm4
       vandnps  xmm0, xmm4, xmm0
       vorps    xmm0, xmm0, xmm12
       vmovsd   xmm12, qword ptr [rbp-0x2A0]
       vandps   xmm12, xmm12, xmm4
       vandnps  xmm3, xmm4, xmm3
       vorps    xmm3, xmm3, xmm12
       vmovss   xmm12, dword ptr [rsi]
       vandps   xmm12, xmm12, xmm4
       vmovss   xmm13, dword ptr [rdi]
       vandnps  xmm13, xmm4, xmm13
       vorps    xmm12, xmm13, xmm12
       vmovss   dword ptr [rbp-0x2A4], xmm12
       vmovss   xmm13, dword ptr [rdi+0x04]
       vandps   xmm13, xmm13, xmm4
       vmovss   xmm15, dword ptr [rsi+0x04]
       vandnps  xmm15, xmm4, xmm15
       vorps    xmm13, xmm15, xmm13
       vmovaps  xmm15, xmm0
 
G_M000_IG25:                ;; offset=0x1C45
       vsubps   xmm3, xmm3, xmm15
       vmovaps  xmm15, xmm3
       vmovaps  xmm12, xmm3
       vmulss   xmm12, xmm15, xmm12
       vmovss   dword ptr [rbp-0x538], xmm12
       vmovshdup xmm15, xmm3
       vmovshdup xmm12, xmm3
       vmulss   xmm12, xmm15, xmm12
       vaddss   xmm12, xmm12, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x464], xmm12
       vdivss   xmm15, xmm6, xmm12
       vmovss   dword ptr [rbp-0x468], xmm15
       vmovaps  xmm15, xmm0
       vmovaps  xmm12, xmm3
       vmulss   xmm12, xmm15, xmm12
       vmovss   dword ptr [rbp-0x538], xmm12
       vmovshdup xmm15, xmm0
       vmovshdup xmm12, xmm3
       vmulss   xmm12, xmm15, xmm12
       vaddss   xmm12, xmm12, dword ptr [rbp-0x538]
       vmovss   dword ptr [rbp-0x46C], xmm12
       vmovaps  xmm15, xmm0
       vmovaps  xmm12, xmm0
       vmulss   xmm12, xmm15, xmm12
       vmovss   dword ptr [rbp-0x538], xmm12
       vmovshdup xmm15, xmm0
       vmovshdup xmm12, xmm0
       vmulss   xmm12, xmm15, xmm12
       vaddss   xmm12, xmm12, dword ptr [rbp-0x538]
       vmovss   xmm15, dword ptr [rbp-0x2A4]
       vmulss   xmm15, xmm15, xmm15
       vsubss   xmm12, xmm12, xmm15
       vmovss   dword ptr [rbp-0x470], xmm12
       vmovss   xmm15, dword ptr [rbp-0x46C]
       vmulss   xmm15, xmm15, xmm15
       vmovss   xmm12, dword ptr [rbp-0x464]
       vmulss   xmm12, xmm12, dword ptr [rbp-0x470]
       vsubss   xmm12, xmm15, xmm12
       vxorps   xmm15, xmm15, xmm15
       vmaxss   xmm12, xmm15, xmm12
       vsqrtss  xmm12, xmm12, xmm12
       vmulss   xmm12, xmm12, dword ptr [rbp-0x468]
       vmovss   dword ptr [rbp-0x474], xmm12
       vmovss   xmm12, dword ptr [rbp-0x46C]
       vxorps   xmm12, xmm12, xmmword ptr [reloc @RWD16]
       vmulss   xmm12, xmm12, dword ptr [rbp-0x468]
       vmovss   dword ptr [rbp-0x478], xmm12
       vsubss   xmm15, xmm12, dword ptr [rbp-0x474]
       vaddss   xmm12, xmm12, dword ptr [rbp-0x474]
       vmovss   xmm1, dword ptr [rbp-0x464]
       vandps   xmm1, xmm1, xmmword ptr [reloc @RWD48]
       vbroadcastss xmm1, xmm1
       vcmpltps xmm1, xmm1, xmmword ptr [reloc @RWD112]
       vmovss   xmm8, dword ptr [rbp-0x478]
       vandps   xmm8, xmm8, xmm1
       vandnps  xmm15, xmm1, xmm15
       vorps    xmm15, xmm15, xmm8
       vmovss   xmm8, dword ptr [rbp-0x478]
       vandps   xmm8, xmm8, xmm1
       vandnps  xmm1, xmm1, xmm12
       vorps    xmm12, xmm1, xmm8
       vxorps   xmm1, xmm13, xmmword ptr [reloc @RWD16]
       vmaxss   xmm1, xmm1, xmm15
 
G_M000_IG26:                ;; offset=0x1DC7
       vmovaps  xmm15, xmm13
       vminss   xmm15, xmm15, xmm1
       vminss   xmm12, xmm13, xmm12
       vmovaps  xmm1, xmm2
       vinsertps xmm1, xmm1, xmm15, 16
       vunpckhps xmm8, xmm2, xmm2
       vinsertps xmm1, xmm1, xmm8, 40
       vmovaps  xmmword ptr [rbp-0x2C0], xmm1
       vmovaps  xmm8, xmm3
       vmulss   xmm8, xmm15, xmm8
       vmovaps  xmm13, xmm0
       vaddss   xmm8, xmm8, xmm13
       vinsertps xmm8, xmm8, xmm11, 16
       vmovshdup xmm13, xmm3
       vmulss   xmm13, xmm15, xmm13
       vmovshdup xmm1, xmm0
       vaddss   xmm1, xmm13, xmm1
       vinsertps xmm1, xmm8, xmm1, 40
       vmovaps  xmm8, xmmword ptr [rbp-0x2C0]
       vandps   xmm8, xmm8, xmm4
       vandnps  xmm1, xmm4, xmm1
       vorps    xmm1, xmm1, xmm8
       vmovaps  xmm8, xmm2
       vinsertps xmm8, xmm8, xmm12, 16
       vunpckhps xmm2, xmm2, xmm2
       vinsertps xmm2, xmm8, xmm2, 40
       vmovaps  xmm8, xmm3
       vmulss   xmm8, xmm12, xmm8
       vmovaps  xmm13, xmm0
       vaddss   xmm8, xmm8, xmm13
       vinsertps xmm8, xmm8, xmm11, 16
       vmovshdup xmm3, xmm3
       vmulss   xmm3, xmm12, xmm3
       vmovshdup xmm0, xmm0
       vaddss   xmm0, xmm3, xmm0
       vinsertps xmm0, xmm8, xmm0, 40
       vandps   xmm2, xmm2, xmm4
       vandnps  xmm0, xmm4, xmm0
       vorps    xmm0, xmm0, xmm2
       mov      byte  ptr [rbx+0x5C], 1
       vucomiss xmm12, xmm15
       seta     dl
       mov      byte  ptr [rbx+0x5D], dl
       vmovaps  xmm2, xmmword ptr [rbp-0x1A0]
       vandps   xmm2, xmm2, xmm4
       vmovaps  xmm3, xmmword ptr [rbp-0x1B0]
       vmovaps  xmm8, xmm3
       vandnps  xmm8, xmm4, xmm8
       vorps    xmm2, xmm8, xmm2
       vandps   xmm8, xmm14, xmm4
       vmovaps  xmm12, xmmword ptr [rbp-0x1C0]
       vmovaps  xmm11, xmm12
       vandnps  xmm4, xmm4, xmm11
       vorps    xmm4, xmm4, xmm8
       vmovaps  xmm3, xmm2
       vmovaps  xmm14, xmm1
       vmovaps  xmm1, xmm12
       vmovaps  xmm12, xmm4
       vmovaps  xmm15, xmm0
 
G_M000_IG27:                ;; offset=0x1EE3
       test     eax, eax
       sete     al
       movzx    rax, al
       test     ecx, ecx
       sete     cl
       movzx    rcx, cl
       test     eax, ecx
       je       G_M000_IG29
       vmovaps  xmm3, xmm1
       vxorps   xmm2, xmm3, xmmword ptr [reloc @RWD16]
       vmovss   xmm3, dword ptr [rbp-0x120]
       vmulss   xmm3, xmm3, dword ptr [rbp-0x120]
       vmovss   xmm4, dword ptr [rbp-0x118]
       vmulss   xmm4, xmm4, dword ptr [rbp-0x118]
       vaddss   xmm3, xmm3, xmm4
       vdivss   xmm3, xmm6, xmm3
       vmovss   xmm4, dword ptr [rsi+0x04]
       vmovss   xmm14, dword ptr [rdi+0x04]
       vmovups  xmm15, xmmword ptr [rbp-0xDC]
       vinsertps xmm12, xmm15, xmm15, 56
       vmovaps  xmm11, xmm2
       vinsertps xmm11, xmm11, xmm11, 56
       vmulps   xmm11, xmm11, xmm12
       vpermilps xmm12, xmm11, -11
       vaddps   xmm12, xmm12, xmm11
       vpermilps xmm11, xmm11, -86
       vaddps   xmm11, xmm11, xmm12
       vmovshdup xmm2, xmm2
       vmovss   xmm12, dword ptr [rbp-0xD8]
       vmulss   xmm13, xmm2, xmm12
       vsubss   xmm11, xmm11, xmm13
       vmulss   xmm13, xmm12, xmm12
       vsubss   xmm13, xmm6, xmm13
       vbroadcastss xmm15, dword ptr [reloc @RWD136]
       vmaxss   xmm13, xmm15, xmm13
       vdivss   xmm11, xmm11, xmm13
       vmulss   xmm11, xmm11, xmm12
       vsubss   xmm11, xmm11, xmm2
       vandps   xmm12, xmm12, xmmword ptr [reloc @RWD48]
       vmulss   xmm4, xmm12, xmm4
       vxorps   xmm12, xmm14, xmmword ptr [reloc @RWD16]
       vxorps   xmm13, xmm4, xmmword ptr [reloc @RWD16]
       vsubss   xmm13, xmm13, xmm2
       vmovaps  xmm15, xmm14
       vminss   xmm13, xmm15, xmm13
       vmaxss   xmm12, xmm12, xmm13
       vxorps   xmm13, xmm14, xmmword ptr [reloc @RWD16]
       vsubss   xmm2, xmm4, xmm2
       vmaxss   xmm2, xmm13, xmm2
       vminss   xmm2, xmm14, xmm2
       vmaxss   xmm4, xmm11, xmm12
       vminss   xmm11, xmm4, xmm2
       vmovss   xmm4, dword ptr [rbp-0xDC]
       vmulss   xmm4, xmm4, dword ptr [rbp-0x118]
       vmovss   xmm13, dword ptr [rbp-0xD4]
       vmulss   xmm13, xmm13, dword ptr [rbp-0x120]
       vsubss   xmm4, xmm4, xmm13
       vmulss   xmm4, xmm4, xmm4
       vmulss   xmm3, xmm4, xmm3
       vmovss   xmm4, dword ptr [reloc @RWD140]
 
G_M000_IG28:                ;; offset=0x2029
       vsubss   xmm3, xmm4, xmm3
       vmulss   xmm3, xmm3, dword ptr [reloc @RWD144]
       vbroadcastss xmm4, dword ptr [reloc @RWD00]
       vminss   xmm3, xmm4, xmm3
       vxorps   xmm4, xmm4, xmm4
       vmaxss   xmm3, xmm4, xmm3
       vmulss   xmm4, xmm11, xmm3
       vsubss   xmm4, xmm11, xmm4
       vmulss   xmm11, xmm3, xmm12
       vaddss   xmm11, xmm11, xmm4
       vmulss   xmm2, xmm3, xmm2
       vaddss   xmm2, xmm2, xmm4
       vmovaps  xmm3, xmm5
       vinsertps xmm3, xmm3, xmm11, 16
       vmovshdup xmm4, xmm5
       vinsertps xmm14, xmm3, xmm4, 40
       vmovaps  xmm3, xmm5
       vinsertps xmm3, xmm3, xmm2, 16
       vmovshdup xmm4, xmm5
       vinsertps xmm15, xmm3, xmm4, 40
       mov      byte  ptr [rbx+0x5C], 1
       vucomiss xmm2, xmm11
       seta     al
       mov      byte  ptr [rbx+0x5D], al
       vmovaps  xmm0, xmmword ptr [rbp-0x1B0]
       vmovaps  xmm3, xmm0
       vmovaps  xmm12, xmm1
 
G_M000_IG29:                ;; offset=0x20AA
       vmovaps  xmm0, xmm3
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
       vmovaps  xmm4, xmm12
       vsubps   xmm2, xmm2, xmm4
       vinsertps xmm2, xmm2, xmm2, 56
       vmovaps  xmm4, xmm3
       vinsertps xmm4, xmm4, xmm4, 56
       vmulps   xmm2, xmm4, xmm2
       vpermilps xmm4, xmm2, -11
       vaddps   xmm4, xmm4, xmm2
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm4
       vmulss   xmm2, xmm2, xmm0
       vmovss   dword ptr [rax], xmm2
       vmovaps  xmm8, xmmword ptr [rbp-0x100]
       vmovaps  xmm2, xmm8
       vaddps   xmm2, xmm2, xmm14
       vmovaps  xmm4, xmm9
       vmovaps  xmm5, xmm2
       vbroadcastss xmm5, xmm5
       vmulps   xmm4, xmm5, xmm4
       vmovaps  xmm5, xmm10
       vmovshdup xmm6, xmm2
       vbroadcastss xmm6, xmm6
       vmulps   xmm5, xmm6, xmm5
       vaddps   xmm4, xmm5, xmm4
       vmovaps  xmm5, xmm7
       vunpckhps xmm2, xmm2, xmm2
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm5
       vaddps   xmm2, xmm2, xmm4
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
       vmovaps  xmm4, xmm12
       vsubps   xmm2, xmm2, xmm4
       vinsertps xmm2, xmm2, xmm2, 56
       vmovaps  xmm4, xmm3
       vinsertps xmm4, xmm4, xmm4, 56
       vmulps   xmm2, xmm4, xmm2
       vpermilps xmm4, xmm2, -11
       vaddps   xmm4, xmm4, xmm2
 
G_M000_IG30:                ;; offset=0x21D3
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm4
       vmulss   xmm2, xmm2, xmm0
       vmovss   dword ptr [rcx], xmm2
       vmovaps  xmm2, xmm8
       vaddps   xmm2, xmm2, xmm15
       vmovaps  xmm4, xmm9
       vmovaps  xmm5, xmm2
       vbroadcastss xmm5, xmm5
       vmulps   xmm4, xmm5, xmm4
       vmovaps  xmm5, xmm10
       vmovshdup xmm6, xmm2
       vbroadcastss xmm6, xmm6
       vmulps   xmm5, xmm6, xmm5
       vaddps   xmm4, xmm5, xmm4
       vmovaps  xmm5, xmm7
       vunpckhps xmm2, xmm2, xmm2
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm5
       vaddps   xmm2, xmm2, xmm4
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
       vmovaps  xmm2, xmmword ptr [rbp-0x170]
       vmovaps  xmm4, xmm2
       vmovaps  xmm5, xmm12
       vsubps   xmm4, xmm4, xmm5
       vinsertps xmm4, xmm4, xmm4, 56
       vmovaps  xmm5, xmm3
       vinsertps xmm5, xmm5, xmm5, 56
       vmulps   xmm4, xmm5, xmm4
       vpermilps xmm5, xmm4, -11
       vaddps   xmm5, xmm5, xmm4
       vpermilps xmm4, xmm4, -86
       vaddps   xmm4, xmm4, xmm5
       vmulss   xmm4, xmm4, xmm0
       vmovss   dword ptr [rcx], xmm4
       vmovaps  xmm4, xmm8
       vaddps   xmm2, xmm4, xmm2
       vmovaps  xmm4, xmm9
       vmovaps  xmm5, xmm2
       vbroadcastss xmm5, xmm5
       vmulps   xmm4, xmm5, xmm4
       vmovaps  xmm5, xmm10
       vmovshdup xmm6, xmm2
       vbroadcastss xmm6, xmm6
       vmulps   xmm5, xmm6, xmm5
       vaddps   xmm4, xmm5, xmm4
       vmovaps  xmm5, xmm7
       vunpckhps xmm2, xmm2, xmm2
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm5
       vaddps   xmm2, xmm2, xmm4
       vmovsd   qword ptr [rax], xmm2
       vextractps dword ptr [rax+0x08], xmm2, 2
       vmovss   xmm2, dword ptr [rcx]
       vucomiss xmm2, xmm1
       setae    al
       movzx    rax, al
       and      byte  ptr [rdx], al
 
G_M000_IG31:                ;; offset=0x22F9
       lea      rax, bword ptr [rbx+0x24]
       lea      rcx, bword ptr [rbx+0x48]
       lea      rdx, bword ptr [rbx+0x5F]
       vmovaps  xmm2, xmmword ptr [rbp-0x180]
       vmovaps  xmm4, xmm2
       vsubps   xmm4, xmm4, xmm12
       vinsertps xmm4, xmm4, xmm4, 56
       vinsertps xmm3, xmm3, xmm3, 56
       vmulps   xmm3, xmm3, xmm4
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
 
G_M000_IG32:                ;; offset=0x2400
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
RWD112 	dq	2B8CBCCC2B8CBCCCh, 2B8CBCCC2B8CBCCCh
RWD128 	dd	3F000000h		;       0.5
RWD132 	dd	469C395Dh		;   19996.7
RWD136 	dd	26901D7Dh		;     1e-15
RWD140 	dd	3CB851ECh		;    0.0225
RWD144 	dd	4234FED7h		;   45.2489

; Total bytes of code 9318

