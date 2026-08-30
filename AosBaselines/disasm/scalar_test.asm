; Assembly listing for method AosBaselines.CylinderPairScalarTester:Test(byref,byref,float,byref,byref,byref,byref) (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rbp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 244 single block inlinees; 67 inlinees without PGO data

G_M000_IG01:                ;; offset=0x0000
       push     rbp
       push     r14
       push     rdi
       push     rsi
       push     rbx
       sub      rsp, 0x5D0
       vmovaps  xmmword ptr [rsp+0x5C0], xmm6
       vmovaps  xmmword ptr [rsp+0x5B0], xmm7
       vmovaps  xmmword ptr [rsp+0x5A0], xmm8
       vmovaps  xmmword ptr [rsp+0x590], xmm9
       vmovaps  xmmword ptr [rsp+0x580], xmm10
       vmovaps  xmmword ptr [rsp+0x570], xmm11
       vmovaps  xmmword ptr [rsp+0x560], xmm12
       vmovaps  xmmword ptr [rsp+0x550], xmm13
       vmovaps  xmmword ptr [rsp+0x540], xmm14
       vmovaps  xmmword ptr [rsp+0x530], xmm15
       lea      rbp, [rsp+0x5F0]
       vxorps   xmm4, xmm4, xmm4
       vmovdqu  ymmword ptr [rbp-0x140], ymm4
       vmovdqu  ymmword ptr [rbp-0x120], ymm4
       vmovdqu  ymmword ptr [rbp-0x100], ymm4
       vmovdqu  ymmword ptr [rbp-0xE0], ymm4
       mov      rsi, rcx
       mov      rdi, rdx
       mov      rbx, bword ptr [rbp+0x40]
 
G_M000_IG02:                ;; offset=0x009D
       vxorps   ymm0, ymm0, ymm0
       vmovdqu  ymmword ptr [rbx], ymm0
       vmovdqu  ymmword ptr [rbx+0x20], ymm0
       vmovdqu  ymmword ptr [rbx+0x40], ymm0
       mov      r8, bword ptr [rbp+0x30]
       vmovups  xmm0, xmmword ptr [r8]
       vmovaps  xmm1, xmm0
       vaddss   xmm3, xmm1, xmm1
       vmovshdup xmm4, xmm0
       vaddss   xmm5, xmm4, xmm4
       vunpckhps xmm6, xmm0, xmm0
       vaddss   xmm7, xmm6, xmm6
       vmulss   xmm1, xmm3, xmm1
       vmulss   xmm8, xmm5, xmm4
       vmulss   xmm9, xmm7, xmm6
       vmulss   xmm4, xmm3, xmm4
       vmulss   xmm10, xmm3, xmm6
       vshufps  xmm0, xmm0, xmm0, -1
       vmulss   xmm3, xmm3, xmm0
       vmulss   xmm6, xmm5, xmm6
       vmulss   xmm5, xmm5, xmm0
       vmulss   xmm0, xmm7, xmm0
       vmovss   xmm7, dword ptr [reloc @RWD00]
       vsubss   xmm11, xmm7, xmm8
       vsubss   xmm11, xmm11, xmm9
       vaddss   xmm12, xmm4, xmm0
       vinsertps xmm11, xmm11, xmm12, 16
       vsubss   xmm12, xmm10, xmm5
       vinsertps xmm11, xmm11, xmm12, 40
       vsubss   xmm0, xmm4, xmm0
       vsubss   xmm1, xmm7, xmm1
       vsubss   xmm4, xmm1, xmm9
       vinsertps xmm0, xmm0, xmm4, 16
       vaddss   xmm4, xmm6, xmm3
       vinsertps xmm0, xmm0, xmm4, 40
       vaddss   xmm4, xmm10, xmm5
       vsubss   xmm3, xmm6, xmm3
       vinsertps xmm3, xmm4, xmm3, 16
       vsubss   xmm1, xmm1, xmm8
       vinsertps xmm1, xmm3, xmm1, 40
       mov      r8, bword ptr [rbp+0x38]
       vmovups  xmm3, xmmword ptr [r8]
       vmovaps  xmm4, xmm3
       vaddss   xmm5, xmm4, xmm4
       vmovshdup xmm6, xmm3
       vaddss   xmm8, xmm6, xmm6
       vunpckhps xmm9, xmm3, xmm3
       vaddss   xmm10, xmm9, xmm9
       vmulss   xmm4, xmm5, xmm4
       vmulss   xmm12, xmm8, xmm6
       vmulss   xmm13, xmm10, xmm9
       vmulss   xmm6, xmm5, xmm6
       vmulss   xmm14, xmm5, xmm9
       vshufps  xmm3, xmm3, xmm3, -1
       vmulss   xmm5, xmm5, xmm3
       vmulss   xmm9, xmm8, xmm9
       vmulss   xmm8, xmm8, xmm3
       vmulss   xmm3, xmm10, xmm3
 
G_M000_IG03:                ;; offset=0x01A3
       vsubss   xmm10, xmm7, xmm12
       vsubss   xmm10, xmm10, xmm13
       vaddss   xmm15, xmm6, xmm3
       vinsertps xmm10, xmm10, xmm15, 16
       vsubss   xmm15, xmm14, xmm8
       vinsertps xmm10, xmm10, xmm15, 40
       vsubss   xmm3, xmm6, xmm3
       vsubss   xmm4, xmm7, xmm4
       vsubss   xmm6, xmm4, xmm13
       vinsertps xmm3, xmm3, xmm6, 16
       vaddss   xmm6, xmm9, xmm5
       vinsertps xmm6, xmm3, xmm6, 40
       vaddss   xmm3, xmm14, xmm8
       vsubss   xmm5, xmm9, xmm5
       vinsertps xmm3, xmm3, xmm5, 16
       vsubss   xmm4, xmm4, xmm12
       vinsertps xmm8, xmm3, xmm4, 40
       vmovshdup xmm3, xmm10
       vunpckhps xmm4, xmm10, xmm10
       vunpckhps xmm5, xmm6, xmm6
       vmovaps  xmm9, xmm10
       vmovaps  xmm12, xmm6
       vinsertps xmm9, xmm9, xmm12, 16
       vmovaps  xmm12, xmm8
       vinsertps xmm9, xmm9, xmm12, 40
       vmovshdup xmm12, xmm6
       vinsertps xmm3, xmm3, xmm12, 16
       vmovshdup xmm12, xmm8
       vinsertps xmm3, xmm3, xmm12, 40
       vinsertps xmm4, xmm4, xmm5, 16
       vunpckhps xmm5, xmm8, xmm8
       vinsertps xmm4, xmm4, xmm5, 40
       vmovaps  xmm5, xmm11
       vbroadcastss xmm5, xmm5
       vmulps   xmm5, xmm5, xmm9
       vmovshdup xmm12, xmm11
       vbroadcastss xmm12, xmm12
       vmulps   xmm12, xmm12, xmm3
       vaddps   xmm5, xmm12, xmm5
       vunpckhps xmm11, xmm11, xmm11
       vbroadcastss xmm11, xmm11
       vmulps   xmm11, xmm11, xmm4
       vaddps   xmm5, xmm11, xmm5
       vmovsd   qword ptr [rbp-0xE8], xmm5
       vextractps dword ptr [rbp-0xE0], xmm5, 2
       vmovaps  xmm5, xmm0
       vbroadcastss xmm5, xmm5
       vmulps   xmm5, xmm5, xmm9
       vmovshdup xmm11, xmm0
       vbroadcastss xmm11, xmm11
       vmulps   xmm11, xmm11, xmm3
       vaddps   xmm5, xmm11, xmm5
       vunpckhps xmm0, xmm0, xmm0
       vbroadcastss xmm0, xmm0
       vmulps   xmm0, xmm0, xmm4
       vaddps   xmm0, xmm0, xmm5
       vmovsd   qword ptr [rbp-0xDC], xmm0
       vextractps dword ptr [rbp-0xD4], xmm0, 2
       vmovaps  xmm0, xmm1
       vbroadcastss xmm0, xmm0
       vmulps   xmm0, xmm0, xmm9
 
G_M000_IG04:                ;; offset=0x02DC
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
       vmovaps  xmm1, xmm10
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm1, xmm1, xmm0
       vpermilps xmm3, xmm1, -11
       vaddps   xmm3, xmm3, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm3
       vmovaps  xmm3, xmm6
       vinsertps xmm3, xmm3, xmm3, 56
       vmulps   xmm3, xmm3, xmm0
       vpermilps xmm4, xmm3, -11
       vaddps   xmm4, xmm4, xmm3
       vpermilps xmm3, xmm3, -86
       vaddps   xmm3, xmm3, xmm4
       vmovaps  xmm4, xmm8
       vinsertps xmm4, xmm4, xmm4, 56
       vmulps   xmm0, xmm4, xmm0
       vinsertps xmm1, xmm1, xmm3, 16
       vpermilps xmm3, xmm0, -11
       vaddps   xmm3, xmm3, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm3
       vinsertps xmm9, xmm1, xmm0, 40
       vmovaps  xmmword ptr [rbp-0x100], xmm9
       vmovaps  xmm0, xmm9
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
       vdivss   xmm1, xmm7, xmm0
       vbroadcastss xmm1, xmm1
       vmovaps  xmm3, xmmword ptr [rbp-0x110]
       vmulps   xmm1, xmm1, xmm3
       vmovaps  xmmword ptr [rbp-0x120], xmm1
       vmovss   xmm1, dword ptr [reloc @RWD32]
       vucomiss xmm1, xmm0
       jbe      SHORT G_M000_IG06
 
G_M000_IG05:                ;; offset=0x0408
       vmovaps  xmm0, xmmword ptr [rbp-0x120]
       vxorps   xmm1, xmm1, xmm1
       vinsertps xmm0, xmm0, xmm1, 1
       vmovaps  xmmword ptr [rbp-0x120], xmm0
       vmovaps  xmm0, xmmword ptr [rbp-0x120]
       vmovaps  xmm1, xmm7
       vinsertps xmm0, xmm0, xmm1, 16
       vmovaps  xmmword ptr [rbp-0x120], xmm0
       vmovaps  xmm0, xmmword ptr [rbp-0x120]
       vxorps   xmm1, xmm1, xmm1
       vinsertps xmm0, xmm0, xmm1, 36
       vmovaps  xmmword ptr [rbp-0x120], xmm0
 
G_M000_IG06:                ;; offset=0x0456
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
       ja       G_M000_IG58
 
G_M000_IG07:                ;; offset=0x04ED
       vmovups  xmm0, xmmword ptr [rbp-0xDC]
       vinsertps xmm1, xmm0, xmm0, 56
       vmovaps  xmmword ptr [rbp-0x3B0], xmm1
       vmovaps  xmm2, xmmword ptr [rbp-0x120]
       vmovaps  xmmword ptr [rbp-0x440], xmm2
       vinsertps xmm3, xmm2, xmm2, 56
       vmulps   xmm3, xmm3, xmm1
       vpermilps xmm4, xmm3, -11
       vaddps   xmm4, xmm4, xmm3
       vpermilps xmm3, xmm3, -86
       vaddps   xmm3, xmm3, xmm4
       vdivss   xmm4, xmm7, xmm3
       vmovss   dword ptr [rbp-0x144], xmm4
       vmovss   xmm5, dword ptr [rbp-0x11C]
       vdivss   xmm11, xmm7, xmm5
       vmovss   dword ptr [rbp-0x148], xmm11
       vmovss   xmm12, dword ptr [rsi+0x04]
       vxorps   xmm13, xmm12, xmmword ptr [reloc @RWD16]
       vxorps   xmm14, xmm14, xmm14
       mov      eax, -1
       xor      ecx, ecx
       vucomiss xmm3, xmm14
       cmovbe   eax, ecx
       vmovd    xmm14, eax
       vandps   xmm13, xmm13, xmm14
       vandnps  xmm12, xmm14, xmm12
       vorps    xmm12, xmm12, xmm13
       vbroadcastss xmm12, xmm12
       vmulps   xmm12, xmm12, xmm0
       vmovaps  xmm13, xmmword ptr [rbp-0x110]
       vmovaps  xmmword ptr [rbp-0x450], xmm13
       vaddps   xmm12, xmm12, xmm13
       vmovaps  xmmword ptr [rbp-0x160], xmm12
       vxorps   xmm14, xmm14, xmm14
       vucomiss xmm14, xmm5
       seta     al
       movzx    rax, al
       vmovss   xmm14, dword ptr [rdi+0x04]
       vxorps   xmm15, xmm14, xmmword ptr [reloc @RWD16]
       mov      ecx, -1
       xor      edx, edx
       test     eax, eax
       cmove    ecx, edx
       vmovd    xmm12, ecx
       vandps   xmm15, xmm15, xmm12
       vandnps  xmm12, xmm12, xmm14
       vorps    xmm12, xmm12, xmm15
       vandps   xmm14, xmm3, xmmword ptr [reloc @RWD48]
       vmovss   dword ptr [rbp-0x424], xmm14
       vucomiss xmm14, dword ptr [reloc @RWD64]
       seta     al
       movzx    rax, al
       vandps   xmm5, xmm5, xmmword ptr [reloc @RWD48]
       vmovss   dword ptr [rbp-0x398], xmm5
       vucomiss xmm5, dword ptr [reloc @RWD64]
       seta     cl
       movzx    rcx, cl
       vxorps   xmm15, xmm15, xmm15
       vxorps   xmm5, xmm5, xmm5
       vxorps   xmm14, xmm14, xmm14
       vmovaps  xmmword ptr [rbp-0x170], xmm14
       vxorps   xmm13, xmm13, xmm13
       vmovaps  xmmword ptr [rbp-0x180], xmm13
       vmovaps  xmm13, xmmword ptr [rbp-0x140]
 
G_M000_IG08:                ;; offset=0x064D
       vmovss   xmm14, dword ptr [rbp-0x124]
       vxorps   xmm11, xmm14, xmmword ptr [reloc @RWD16]
       vbroadcastss xmm11, xmm11
       vmulps   xmm11, xmm11, xmm2
       vaddps   xmm11, xmm11, xmm13
       vsubps   xmm11, xmm11, xmmword ptr [rbp-0x450]
       vmovaps  xmm13, xmm11
       vinsertps xmm13, xmm13, xmm13, 56
       vmulps   xmm13, xmm13, xmm1
       vpermilps xmm14, xmm13, -11
       vaddps   xmm14, xmm14, xmm13
       vpermilps xmm13, xmm13, -86
       vaddps   xmm13, xmm13, xmm14
       vbroadcastss xmm13, xmm13
       vmulps   xmm0, xmm13, xmm0
       vsubps   xmm11, xmm11, xmm0
       vmovaps  xmmword ptr [rbp-0x190], xmm11
       vxorps   xmm0, xmm0, xmm0
       vmovss   xmm13, dword ptr [rbp-0x140]
       vinsertps xmm0, xmm0, xmm13, 14
       vmovss   xmm13, dword ptr [rbp-0x138]
       vinsertps xmm0, xmm0, xmm13, 16
       vxorps   xmm13, xmm13, xmm13
       vucomiss xmm3, xmm13
       seta     dl
       movzx    rdx, dl
       vmovss   xmm3, dword ptr [rbp-0xDC]
       vxorps   xmm13, xmm3, xmmword ptr [reloc @RWD16]
       mov      r8d, -1
       xor      r10d, r10d
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm14, r8d
       vandps   xmm13, xmm13, xmm14
       vandnps  xmm3, xmm14, xmm3
       vorps    xmm3, xmm3, xmm13
       vinsertps xmm3, xmm15, xmm3, 0
       vmovss   xmm13, dword ptr [rbp-0xD8]
       vxorps   xmm14, xmm13, xmmword ptr [reloc @RWD16]
       vmovss   dword ptr [rbp-0x274], xmm13
       mov      r8d, -1
 
G_M000_IG09:                ;; offset=0x0733
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm13, r8d
       vandps   xmm14, xmm14, xmm13
       vmovups  xmmword ptr [rbp-0x580], xmm14
       vmovss   xmm14, dword ptr [rbp-0x274]
       vandnps  xmm13, xmm13, xmm14
       vorps    xmm13, xmm13, xmmword ptr [rbp-0x580]
       vinsertps xmm3, xmm3, xmm13, 16
       mov      r8d, eax
       and      r8d, ecx
       vmovss   xmm13, dword ptr [rbp-0xD4]
       vmovaps  xmm14, xmm13
       vmovss   dword ptr [rbp-0x278], xmm13
       mov      r10d, -1
       xor      r9d, r9d
       test     edx, edx
       cmove    r10d, r9d
       vmovd    xmm13, r10d
       vmovaps  xmmword ptr [rbp-0x4F0], xmm13
       vxorps   xmm13, xmm14, xmmword ptr [reloc @RWD16]
       vandps   xmm13, xmm13, xmmword ptr [rbp-0x4F0]
       vmovups  xmmword ptr [rbp-0x580], xmm13
       vmovss   xmm14, dword ptr [rbp-0x278]
       vmovaps  xmm13, xmmword ptr [rbp-0x4F0]
       vandnps  xmm13, xmm13, xmm14
       vorps    xmm13, xmm13, xmmword ptr [rbp-0x580]
       vinsertps xmm3, xmm3, xmm13, 32
       vmovsd   qword ptr [rbp-0x1A0], xmm3
       vextractps dword ptr [rbp-0x198], xmm3, 2
       vmovaps  xmmword ptr [rbp-0x1B0], xmm3
       vmovaps  xmm14, xmmword ptr [rbp-0x160]
       test     r8d, r8d
       jne      SHORT G_M000_IG11
 
G_M000_IG10:                ;; offset=0x07FF
       jmp      G_M000_IG35
 
G_M000_IG11:                ;; offset=0x0804
       vmovss   xmm3, dword ptr [rbp-0x424]
       vmovss   dword ptr [rbp-0x1C4], xmm3
       vmovss   xmm3, dword ptr [rbp-0x398]
       vmovss   dword ptr [rbp-0x1C8], xmm3
       vmovss   xmm3, dword ptr [reloc @RWD68]
       vmovss   dword ptr [rbp-0x4A4], xmm3
       vucomiss xmm3, dword ptr [rbp-0x1C8]
       seta     dl
       movzx    rdx, dl
       vmovaps  xmm3, xmm0
       vmovss   xmm11, dword ptr [rbp-0x4A4]
       vucomiss xmm11, dword ptr [rbp-0x1C4]
       seta     r8b
       movzx    r8, r8b
       test     r8d, edx
       jne      G_M000_IG34
       vmovshdup xmm3, xmm14
       vsubss   xmm5, xmm3, xmm12
       vmulss   xmm5, xmm5, dword ptr [rbp-0x148]
       vmovss   dword ptr [rbp-0x27C], xmm5
       vxorps   xmm3, xmm3, xmm3
       vmulss   xmm11, xmm5, dword ptr [rbp-0x120]
       vmovaps  xmm5, xmm14
       vsubss   xmm5, xmm5, xmm11
       vinsertps xmm3, xmm3, xmm5, 14
       vmovss   xmm5, dword ptr [rbp-0x27C]
       vmulss   xmm5, xmm5, dword ptr [rbp-0x118]
       vunpckhps xmm11, xmm14, xmm14
       vsubss   xmm5, xmm11, xmm5
       vinsertps xmm3, xmm3, xmm5, 16
       vmovaps  xmm5, xmm3
       vmovaps  xmm11, xmm3
       vmulss   xmm5, xmm5, xmm11
       vmovss   dword ptr [rbp-0x564], xmm5
       vmovshdup xmm11, xmm3
       vmovshdup xmm5, xmm3
       vmulss   xmm5, xmm11, xmm5
       vaddss   xmm5, xmm5, dword ptr [rbp-0x564]
       vsqrtss  xmm5, xmm5, xmm5
       vmovss   dword ptr [rbp-0x1CC], xmm5
       vdivss   xmm11, xmm7, xmm5
       vmovss   dword ptr [rbp-0x1D0], xmm11
       vbroadcastss xmm11, xmm11
       vmulps   xmm3, xmm11, xmm3
       vmovss   xmm11, dword ptr [reloc @RWD72]
       vucomiss xmm11, xmm5
       seta     dl
       movzx    rdx, dl
       vmovaps  xmm11, xmm3
       vmovss   dword ptr [rbp-0x280], xmm11
       mov      r8d, -1
       xor      r10d, r10d
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm11, r8d
       vmovaps  xmmword ptr [rbp-0x500], xmm11
       vbroadcastss xmm5, dword ptr [reloc @RWD00]
       vmovaps  xmmword ptr [rbp-0x4C0], xmm5
       vandps   xmm11, xmm5, xmm11
       vmovups  xmmword ptr [rbp-0x580], xmm11
       vmovss   xmm11, dword ptr [rbp-0x280]
       vmovups  xmmword ptr [rbp-0x590], xmm11
       vmovaps  xmm11, xmmword ptr [rbp-0x500]
       vandnps  xmm11, xmm11, xmmword ptr [rbp-0x590]
 
G_M000_IG12:                ;; offset=0x097E
       vorps    xmm11, xmm11, xmmword ptr [rbp-0x580]
       vinsertps xmm3, xmm3, xmm11, 0
       vmovshdup xmm11, xmm3
       vmovss   dword ptr [rbp-0x284], xmm11
       mov      r8d, -1
 
G_M000_IG13:                ;; offset=0x099E
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm11, r8d
       vmovaps  xmmword ptr [rbp-0x510], xmm11
       vxorps   xmm11, xmm11, xmm11
       vandps   xmm11, xmm11, xmmword ptr [rbp-0x510]
       vmovups  xmmword ptr [rbp-0x580], xmm11
       vmovss   xmm11, dword ptr [rbp-0x284]
       vmovups  xmmword ptr [rbp-0x590], xmm11
       vmovaps  xmm11, xmmword ptr [rbp-0x510]
       vandnps  xmm11, xmm11, xmmword ptr [rbp-0x590]
       vorps    xmm11, xmm11, xmmword ptr [rbp-0x580]
       vinsertps xmm3, xmm3, xmm11, 16
       vmovaps  xmm11, xmm3
       vmovups  xmmword ptr [rbp-0x580], xmm11
       vmovss   xmm11, dword ptr [rdi]
       vmovss   dword ptr [rbp-0x4A8], xmm11
       vbroadcastss xmm11, xmm11
       vmulps   xmm11, xmm11, xmmword ptr [rbp-0x580]
       vmovsd   qword ptr [rbp-0x1D8], xmm11
       vmovaps  xmmword ptr [rbp-0x4E0], xmm11
       vxorps   xmm11, xmm11, xmmword ptr [reloc @RWD16]
       vmovsd   qword ptr [rbp-0x1E0], xmm11
       vmovss   xmm11, dword ptr [rbp-0x1D8]
       vinsertps xmm11, xmm15, xmm11, 0
       vmovups  xmmword ptr [rbp-0x580], xmm11
       vmovaps  xmm11, xmm12
       vmovups  xmm5, xmmword ptr [rbp-0x580]
       vinsertps xmm5, xmm5, xmm11, 16
       vmovsd   xmm11, qword ptr [rbp-0x1D8]
       vmovshdup xmm11, xmm11
       vinsertps xmm5, xmm5, xmm11, 32
       vmovaps  xmm11, xmm14
       vmovaps  xmmword ptr [rbp-0x460], xmm5
       vsubps   xmm5, xmm11, xmm5
       vinsertps xmm5, xmm5, xmm5, 56
       vmulps   xmm5, xmm5, xmm1
       vmovaps  xmmword ptr [rbp-0x2A0], xmm5
       vpermilps xmm5, xmm5, -11
       vaddps   xmm5, xmm5, xmmword ptr [rbp-0x2A0]
       vmovups  xmmword ptr [rbp-0x580], xmm5
       vpermilps xmm5, xmmword ptr [rbp-0x2A0], -86
       vaddps   xmm5, xmm5, xmmword ptr [rbp-0x580]
       vmulss   xmm5, xmm5, xmm4
       vbroadcastss xmm5, xmm5
       vmulps   xmm5, xmm5, xmm2
       vaddps   xmm5, xmm5, xmmword ptr [rbp-0x460]
       vsubps   xmm5, xmm5, xmm11
       vinsertps xmm5, xmm5, xmm5, 56
       vmovaps  xmmword ptr [rbp-0x3F0], xmm5
       vmovups  xmm5, xmmword ptr [rbp-0xE8]
       vinsertps xmm5, xmm5, xmm5, 56
       vmovaps  xmmword ptr [rbp-0x3C0], xmm5
       vmulps   xmm5, xmm5, xmmword ptr [rbp-0x3F0]
       vmovaps  xmmword ptr [rbp-0x2B0], xmm5
       vxorps   xmm5, xmm5, xmm5
       vmovsd   qword ptr [rbp-0x570], xmm5
       vpermilps xmm5, xmmword ptr [rbp-0x2B0], -11
       vaddps   xmm5, xmm5, xmmword ptr [rbp-0x2B0]
       vmovups  xmmword ptr [rbp-0x580], xmm5
 
G_M000_IG14:                ;; offset=0x0B34
       vpermilps xmm5, xmmword ptr [rbp-0x2B0], -86
       vaddps   xmm5, xmm5, xmmword ptr [rbp-0x580]
       vmovsd   xmm2, qword ptr [rbp-0x570]
       vinsertps xmm2, xmm2, xmm5, 14
       vmovaps  xmm5, xmmword ptr [rbp-0xD0]
       vinsertps xmm5, xmm5, xmm5, 56
       vmovaps  xmmword ptr [rbp-0x3D0], xmm5
       vmulps   xmm5, xmm5, xmmword ptr [rbp-0x3F0]
       vmovaps  xmmword ptr [rbp-0x2C0], xmm5
       vpermilps xmm5, xmm5, -11
       vaddps   xmm5, xmm5, xmmword ptr [rbp-0x2C0]
       vmovups  xmmword ptr [rbp-0x580], xmm5
       vpermilps xmm5, xmmword ptr [rbp-0x2C0], -86
       vaddps   xmm5, xmm5, xmmword ptr [rbp-0x580]
       vinsertps xmm2, xmm2, xmm5, 16
       vmovss   xmm5, dword ptr [rbp-0x1E0]
       vinsertps xmm5, xmm15, xmm5, 0
       vmovups  xmmword ptr [rbp-0x580], xmm5
       vmovaps  xmm5, xmm12
       vmovups  xmm4, xmmword ptr [rbp-0x580]
       vinsertps xmm4, xmm4, xmm5, 16
       vmovsd   xmm5, qword ptr [rbp-0x1E0]
       vmovshdup xmm5, xmm5
       vinsertps xmm4, xmm4, xmm5, 32
       vmovaps  xmmword ptr [rbp-0x470], xmm4
       vsubps   xmm5, xmm11, xmm4
       vinsertps xmm5, xmm5, xmm5, 56
       vmulps   xmm5, xmm5, xmm1
       vpermilps xmm4, xmm5, -11
       vaddps   xmm4, xmm4, xmm5
       vpermilps xmm5, xmm5, -86
       vaddps   xmm4, xmm5, xmm4
       vmulss   xmm4, xmm4, dword ptr [rbp-0x144]
       vbroadcastss xmm4, xmm4
       vmulps   xmm4, xmm4, xmmword ptr [rbp-0x440]
       vaddps   xmm4, xmm4, xmmword ptr [rbp-0x470]
       vsubps   xmm4, xmm4, xmm11
       vinsertps xmm4, xmm4, xmm4, 56
       vmovaps  xmmword ptr [rbp-0x400], xmm4
       vmovaps  xmm4, xmmword ptr [rbp-0x3C0]
       vmulps   xmm4, xmm4, xmmword ptr [rbp-0x400]
       vxorps   xmm5, xmm5, xmm5
       vmovsd   qword ptr [rbp-0x570], xmm5
       vpermilps xmm5, xmm4, -11
       vaddps   xmm5, xmm5, xmm4
       vpermilps xmm4, xmm4, -86
       vaddps   xmm4, xmm4, xmm5
       vmovsd   xmm5, qword ptr [rbp-0x570]
       vinsertps xmm4, xmm5, xmm4, 14
       vmovsd   qword ptr [rbp-0x1E8], xmm4
       vmovaps  xmm5, xmmword ptr [rbp-0x3D0]
       vmulps   xmm4, xmm5, xmmword ptr [rbp-0x400]
       vpermilps xmm5, xmm4, -11
       vaddps   xmm5, xmm5, xmm4
       vpermilps xmm4, xmm4, -86
       vaddps   xmm4, xmm4, xmm5
       vmovsd   xmm5, qword ptr [rbp-0x1E8]
 
G_M000_IG15:                ;; offset=0x0CAF
       vinsertps xmm4, xmm5, xmm4, 16
       vmovaps  xmm5, xmm2
       vsubps   xmm4, xmm4, xmm5
       vmovsd   qword ptr [rbp-0x1F0], xmm4
       vmovsd   xmm5, qword ptr [rbp-0x1E0]
       vsubps   xmm5, xmm5, xmmword ptr [rbp-0x4E0]
       vmovsd   qword ptr [rbp-0x270], xmm5
       vmovss   xmm4, dword ptr [rsi]
       vmovss   dword ptr [rbp-0x2D4], xmm4
       vmovss   xmm5, dword ptr [rbp-0x1F0]
       vmovss   dword ptr [rbp-0x564], xmm5
       vmovss   xmm5, dword ptr [rbp-0x1F0]
       vmulss   xmm5, xmm5, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm5
       vmovsd   xmm5, qword ptr [rbp-0x1F0]
       vmovshdup xmm5, xmm5
       vmovss   dword ptr [rbp-0x4C4], xmm5
       vmulss   xmm5, xmm5, xmm5
       vaddss   xmm5, xmm5, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x2C4], xmm5
       vdivss   xmm5, xmm7, xmm5
       vmovss   dword ptr [rbp-0x2C8], xmm5
       vmovaps  xmm5, xmm2
       vmovss   dword ptr [rbp-0x564], xmm5
       vmovss   xmm5, dword ptr [rbp-0x1F0]
       vmulss   xmm5, xmm5, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm5
       vmovshdup xmm5, xmm2
       vmovss   xmm4, dword ptr [rbp-0x4C4]
       vmulss   xmm4, xmm5, xmm4
       vaddss   xmm4, xmm4, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x2CC], xmm4
       vmovaps  xmm4, xmm2
       vmulss   xmm2, xmm4, xmm2
       vmulss   xmm4, xmm5, xmm5
       vaddss   xmm2, xmm2, xmm4
       vmovss   xmm4, dword ptr [rbp-0x2D4]
       vmulss   xmm4, xmm4, xmm4
       vmovss   dword ptr [rbp-0x3D4], xmm4
       vsubss   xmm2, xmm2, xmm4
       vmovss   dword ptr [rbp-0x2D0], xmm2
       vmovss   xmm5, dword ptr [rbp-0x2CC]
       vmulss   xmm5, xmm5, xmm5
       vmovss   xmm2, dword ptr [rbp-0x2C4]
       vmulss   xmm2, xmm2, dword ptr [rbp-0x2D0]
       vsubss   xmm2, xmm5, xmm2
       vxorps   xmm5, xmm5, xmm5
       vmaxss   xmm2, xmm5, xmm2
       vsqrtss  xmm2, xmm2, xmm2
       vmulss   xmm2, xmm2, dword ptr [rbp-0x2C8]
       vmovss   dword ptr [rbp-0x2D8], xmm2
       vmovss   xmm2, dword ptr [rbp-0x2CC]
       vxorps   xmm2, xmm2, xmmword ptr [reloc @RWD16]
       vmulss   xmm2, xmm2, dword ptr [rbp-0x2C8]
       vsubss   xmm5, xmm2, dword ptr [rbp-0x2D8]
       vmovss   dword ptr [rbp-0x1F4], xmm5
       vaddss   xmm4, xmm2, dword ptr [rbp-0x2D8]
       vmovss   dword ptr [rbp-0x1F8], xmm4
       vmovss   xmm5, dword ptr [rbp-0x2C4]
       vandps   xmm5, xmm5, xmmword ptr [reloc @RWD48]
 
G_M000_IG16:                ;; offset=0x0E3D
       vmovss   xmm4, dword ptr [reloc @RWD76]
       vucomiss xmm4, xmm5
       seta     dl
       movzx    rdx, dl
       mov      r8d, -1
 
G_M000_IG17:                ;; offset=0x0E55
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm4, r8d
       vmovaps  xmmword ptr [rbp-0x480], xmm2
       vandps   xmm5, xmm2, xmm4
       vmovss   xmm2, dword ptr [rbp-0x1F4]
       vandnps  xmm2, xmm4, xmm2
       vorps    xmm5, xmm2, xmm5
       vmovss   dword ptr [rbp-0x1F4], xmm5
       mov      r8d, -1
 
G_M000_IG18:                ;; offset=0x0E8A
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm2, r8d
       vmovaps  xmm4, xmmword ptr [rbp-0x480]
       vandps   xmm4, xmm4, xmm2
       vmovss   xmm5, dword ptr [rbp-0x1F8]
       vandnps  xmm2, xmm2, xmm5
       vorps    xmm4, xmm2, xmm4
       vmovss   xmm2, dword ptr [rbp-0x1F4]
       vxorps   xmm5, xmm5, xmm5
       vmaxss   xmm2, xmm2, xmm5
       vmovss   dword ptr [rbp-0x1FC], xmm2
       vminss   xmm4, xmm4, dword ptr [rbp-0x4C0]
       vmovss   dword ptr [rbp-0x200], xmm4
       vmovsd   xmm4, qword ptr [rbp-0x270]
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm4
       vaddps   xmm2, xmm2, xmmword ptr [rbp-0x4E0]
       vmovsd   xmm4, qword ptr [rbp-0x270]
       vmovups  xmmword ptr [rbp-0x580], xmm4
       vbroadcastss xmm4, dword ptr [rbp-0x200]
       vmulps   xmm4, xmm4, xmmword ptr [rbp-0x580]
       vaddps   xmm4, xmm4, xmmword ptr [rbp-0x4E0]
       vmovss   xmm5, dword ptr [rbp-0x4A8]
       vmulss   xmm5, xmm5, dword ptr [rbp-0x4A8]
       vsubss   xmm5, xmm5, dword ptr [rbp-0x3D4]
       vmulss   xmm5, xmm5, dword ptr [rbp-0x1D0]
       vaddss   xmm5, xmm5, dword ptr [rbp-0x1CC]
       vmulss   xmm5, xmm5, dword ptr [reloc @RWD80]
       vmovss   dword ptr [rbp-0x204], xmm5
       vmovss   xmm5, dword ptr [rbp-0x1CC]
       vmovups  xmmword ptr [rbp-0x580], xmm5
       vxorps   xmm5, xmm5, xmm5
       vmaxss   xmm5, xmm5, dword ptr [rbp-0x204]
       vmovss   dword ptr [rbp-0x564], xmm5
       vmovups  xmm5, xmmword ptr [rbp-0x580]
       vminss   xmm5, xmm5, dword ptr [rbp-0x564]
       vbroadcastss xmm5, xmm5
       vmovups  xmmword ptr [rbp-0x580], xmm5
       vmovaps  xmm5, xmm3
       vmovups  xmmword ptr [rbp-0x590], xmm5
       vmovups  xmm5, xmmword ptr [rbp-0x580]
       vmulps   xmm5, xmm5, xmmword ptr [rbp-0x590]
       vxorps   xmm1, xmm1, xmm1
       vmovsd   qword ptr [rbp-0x570], xmm1
       vmovshdup xmm1, xmm3
       vmovsd   xmm9, qword ptr [rbp-0x570]
       vinsertps xmm1, xmm9, xmm1, 14
       vmovsd   qword ptr [rbp-0x570], xmm1
       vmovaps  xmm9, xmm3
       vxorps   xmm1, xmm9, xmmword ptr [reloc @RWD16]
       vmovsd   xmm9, qword ptr [rbp-0x570]
       vinsertps xmm1, xmm9, xmm1, 16
       vmovaps  xmm9, xmm5
       vmovups  xmmword ptr [rbp-0x590], xmm9
       vmovaps  xmm9, xmm1
       vaddps   xmm9, xmm9, xmmword ptr [rbp-0x590]
       vmovsd   qword ptr [rbp-0x210], xmm9
       vmovaps  xmm9, xmm5
       vinsertps xmm9, xmm15, xmm9, 0
       vmovups  xmmword ptr [rbp-0x590], xmm9
       vmovaps  xmm9, xmm12
 
G_M000_IG19:                ;; offset=0x1027
       vmovups  xmm13, xmmword ptr [rbp-0x590]
       vinsertps xmm9, xmm13, xmm9, 16
       vmovshdup xmm13, xmm5
       vinsertps xmm9, xmm9, xmm13, 32
       vmovaps  xmmword ptr [rbp-0x490], xmm9
       vsubps   xmm13, xmm11, xmm9
       vinsertps xmm13, xmm13, xmm13, 56
       vmulps   xmm13, xmm13, xmmword ptr [rbp-0x3B0]
       vpermilps xmm9, xmm13, -11
       vaddps   xmm9, xmm9, xmm13
       vpermilps xmm13, xmm13, -86
       vaddps   xmm9, xmm13, xmm9
       vmulss   xmm9, xmm9, dword ptr [rbp-0x144]
       vbroadcastss xmm9, xmm9
       vmulps   xmm9, xmm9, xmmword ptr [rbp-0x440]
       vaddps   xmm9, xmm9, xmmword ptr [rbp-0x490]
       vsubps   xmm9, xmm9, xmm11
       vinsertps xmm9, xmm9, xmm9, 56
       vmovaps  xmmword ptr [rbp-0x410], xmm9
       vmovaps  xmm9, xmmword ptr [rbp-0x3C0]
       vmulps   xmm9, xmm9, xmmword ptr [rbp-0x410]
       vxorps   xmm13, xmm13, xmm13
       vmovsd   qword ptr [rbp-0x570], xmm13
       vpermilps xmm13, xmm9, -11
       vaddps   xmm13, xmm13, xmm9
       vpermilps xmm9, xmm9, -86
       vaddps   xmm9, xmm9, xmm13
       vmovsd   xmm13, qword ptr [rbp-0x570]
       vinsertps xmm9, xmm13, xmm9, 14
       vmovaps  xmm13, xmmword ptr [rbp-0x3D0]
       vmulps   xmm13, xmm13, xmmword ptr [rbp-0x410]
       vmovaps  xmmword ptr [rbp-0x2F0], xmm13
       vpermilps xmm13, xmm13, -11
       vaddps   xmm13, xmm13, xmmword ptr [rbp-0x2F0]
       vmovups  xmmword ptr [rbp-0x590], xmm13
       vpermilps xmm13, xmmword ptr [rbp-0x2F0], -86
       vaddps   xmm13, xmm13, xmmword ptr [rbp-0x590]
       vinsertps xmm9, xmm9, xmm13, 16
       vmovsd   qword ptr [rbp-0x218], xmm9
       vmovss   xmm13, dword ptr [rbp-0x210]
       vinsertps xmm13, xmm15, xmm13, 0
       vmovups  xmmword ptr [rbp-0x590], xmm13
       vmovaps  xmm13, xmm12
       vmovups  xmm9, xmmword ptr [rbp-0x590]
       vinsertps xmm9, xmm9, xmm13, 16
       vmovsd   xmm13, qword ptr [rbp-0x210]
       vmovshdup xmm13, xmm13
       vinsertps xmm9, xmm9, xmm13, 32
       vmovaps  xmmword ptr [rbp-0x4A0], xmm9
       vsubps   xmm13, xmm11, xmm9
       vinsertps xmm13, xmm13, xmm13, 56
       vmulps   xmm13, xmm13, xmmword ptr [rbp-0x3B0]
       vpermilps xmm9, xmm13, -11
       vaddps   xmm9, xmm9, xmm13
       vpermilps xmm13, xmm13, -86
       vaddps   xmm9, xmm13, xmm9
       vmulss   xmm9, xmm9, dword ptr [rbp-0x144]
       vbroadcastss xmm9, xmm9
 
G_M000_IG20:                ;; offset=0x11A9
       vmulps   xmm9, xmm9, xmmword ptr [rbp-0x440]
       vaddps   xmm9, xmm9, xmmword ptr [rbp-0x4A0]
       vsubps   xmm11, xmm9, xmm11
       vinsertps xmm9, xmm11, xmm11, 56
       vmovaps  xmmword ptr [rbp-0x420], xmm9
       vmovaps  xmm11, xmmword ptr [rbp-0x3C0]
       vmulps   xmm11, xmm11, xmm9
       vxorps   xmm9, xmm9, xmm9
       vpermilps xmm13, xmm11, -11
       vaddps   xmm13, xmm13, xmm11
       vpermilps xmm11, xmm11, -86
       vaddps   xmm11, xmm11, xmm13
       vinsertps xmm9, xmm9, xmm11, 14
       vmovaps  xmm13, xmmword ptr [rbp-0x3D0]
       vmulps   xmm11, xmm13, xmmword ptr [rbp-0x420]
       vpermilps xmm13, xmm11, -11
       vaddps   xmm13, xmm13, xmm11
       vpermilps xmm11, xmm11, -86
       vaddps   xmm11, xmm11, xmm13
       vinsertps xmm9, xmm9, xmm11, 16
       vmovsd   xmm11, qword ptr [rbp-0x218]
       vmovaps  xmm13, xmm11
       vsubps   xmm9, xmm9, xmm13
       vmovaps  xmm13, xmm9
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovaps  xmm13, xmm9
       vmulss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovshdup xmm13, xmm9
       vmovss   dword ptr [rbp-0x568], xmm13
       vmovshdup xmm13, xmm9
       vmulss   xmm13, xmm13, dword ptr [rbp-0x568]
       vaddss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x2F4], xmm13
       vdivss   xmm13, xmm7, xmm13
       vmovss   dword ptr [rbp-0x2F8], xmm13
       vmovaps  xmm13, xmm11
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovaps  xmm13, xmm9
       vmulss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovshdup xmm13, xmm11
       vmovshdup xmm9, xmm9
       vmulss   xmm9, xmm13, xmm9
       vaddss   xmm9, xmm9, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x2FC], xmm9
       vmovaps  xmm13, xmm11
       vmovaps  xmm9, xmm11
       vmulss   xmm9, xmm13, xmm9
       vmovshdup xmm13, xmm11
       vmovshdup xmm11, xmm11
       vmulss   xmm11, xmm13, xmm11
       vaddss   xmm9, xmm9, xmm11
       vsubss   xmm9, xmm9, dword ptr [rbp-0x3D4]
       vmovss   xmm11, dword ptr [rbp-0x2FC]
       vmulss   xmm13, xmm11, xmm11
       vmovss   xmm11, dword ptr [rbp-0x2F4]
       vmulss   xmm9, xmm11, xmm9
       vsubss   xmm9, xmm13, xmm9
       vxorps   xmm13, xmm13, xmm13
       vmaxss   xmm9, xmm13, xmm9
       vsqrtss  xmm9, xmm9, xmm9
 
G_M000_IG21:                ;; offset=0x132B
       vmulss   xmm9, xmm9, dword ptr [rbp-0x2F8]
       vmovss   xmm11, dword ptr [rbp-0x2FC]
       vxorps   xmm11, xmm11, xmmword ptr [reloc @RWD16]
       vmulss   xmm11, xmm11, dword ptr [rbp-0x2F8]
       vsubss   xmm13, xmm11, xmm9
       vmovss   dword ptr [rbp-0x21C], xmm13
       vaddss   xmm9, xmm11, xmm9
       vmovss   dword ptr [rbp-0x220], xmm9
       vmovss   xmm13, dword ptr [rbp-0x2F4]
       vandps   xmm13, xmm13, xmmword ptr [reloc @RWD48]
       vmovss   xmm9, dword ptr [reloc @RWD76]
       vucomiss xmm9, xmm13
       seta     dl
       movzx    rdx, dl
       mov      r8d, -1
 
G_M000_IG22:                ;; offset=0x138E
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm9, r8d
       vmovaps  xmm13, xmm11
       vandps   xmm13, xmm13, xmm9
       vmovups  xmmword ptr [rbp-0x590], xmm13
       vmovss   xmm13, dword ptr [rbp-0x21C]
       vandnps  xmm9, xmm9, xmm13
       vorps    xmm13, xmm9, xmmword ptr [rbp-0x590]
       vmovss   dword ptr [rbp-0x21C], xmm13
       mov      r8d, -1
 
G_M000_IG23:                ;; offset=0x13CE
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm9, r8d
       vandps   xmm11, xmm11, xmm9
       vmovss   xmm13, dword ptr [rbp-0x220]
       vandnps  xmm9, xmm9, xmm13
       vorps    xmm9, xmm9, xmm11
       vmovss   dword ptr [rbp-0x220], xmm9
       vmovss   xmm11, dword ptr [rbp-0x4A8]
       vmovaps  xmm13, xmm1
       vmovaps  xmm9, xmm1
       vmulss   xmm9, xmm13, xmm9
       vmovss   dword ptr [rbp-0x564], xmm9
       vmovshdup xmm13, xmm1
       vmovshdup xmm9, xmm1
       vmulss   xmm9, xmm13, xmm9
       vaddss   xmm9, xmm9, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x300], xmm9
       vdivss   xmm13, xmm7, xmm9
       vmovss   dword ptr [rbp-0x304], xmm13
       vmovaps  xmm13, xmm5
       vmovaps  xmm9, xmm1
       vmulss   xmm9, xmm13, xmm9
       vmovss   dword ptr [rbp-0x564], xmm9
       vmovshdup xmm13, xmm5
       vmovshdup xmm9, xmm1
       vmulss   xmm9, xmm13, xmm9
       vaddss   xmm9, xmm9, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x308], xmm9
       vmovaps  xmm13, xmm5
       vmovaps  xmm9, xmm5
       vmulss   xmm9, xmm13, xmm9
       vmovss   dword ptr [rbp-0x564], xmm9
       vmovshdup xmm13, xmm5
       vmovshdup xmm9, xmm5
       vmulss   xmm9, xmm13, xmm9
       vaddss   xmm9, xmm9, dword ptr [rbp-0x564]
       vmulss   xmm11, xmm11, xmm11
       vsubss   xmm9, xmm9, xmm11
       vmovss   xmm11, dword ptr [rbp-0x308]
       vmulss   xmm13, xmm11, xmm11
       vmovss   xmm11, dword ptr [rbp-0x300]
       vmulss   xmm9, xmm11, xmm9
       vsubss   xmm9, xmm13, xmm9
       vxorps   xmm13, xmm13, xmm13
       vmaxss   xmm9, xmm13, xmm9
       vsqrtss  xmm9, xmm9, xmm9
       vmulss   xmm9, xmm9, dword ptr [rbp-0x304]
       vmovss   xmm11, dword ptr [rbp-0x308]
       vxorps   xmm11, xmm11, xmmword ptr [reloc @RWD16]
       vmulss   xmm11, xmm11, dword ptr [rbp-0x304]
       vsubss   xmm13, xmm11, xmm9
       vmovss   dword ptr [rbp-0x224], xmm13
       vaddss   xmm9, xmm11, xmm9
       vmovss   dword ptr [rbp-0x228], xmm9
       vmovss   xmm13, dword ptr [rbp-0x300]
       vandps   xmm13, xmm13, xmmword ptr [reloc @RWD48]
       vmovss   xmm9, dword ptr [reloc @RWD76]
       vucomiss xmm9, xmm13
       seta     dl
       movzx    rdx, dl
       mov      r8d, -1
 
G_M000_IG24:                ;; offset=0x1536
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm9, r8d
       vmovaps  xmm13, xmm11
       vandps   xmm13, xmm13, xmm9
       vmovups  xmmword ptr [rbp-0x590], xmm13
       vmovss   xmm13, dword ptr [rbp-0x224]
       vandnps  xmm9, xmm9, xmm13
       vorps    xmm13, xmm9, xmmword ptr [rbp-0x590]
       vmovss   dword ptr [rbp-0x224], xmm13
       mov      r8d, -1
 
G_M000_IG25:                ;; offset=0x1576
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm9, r8d
       vandps   xmm11, xmm11, xmm9
       vmovss   xmm13, dword ptr [rbp-0x228]
       vandnps  xmm9, xmm9, xmm13
       vorps    xmm9, xmm9, xmm11
       vmovss   xmm11, dword ptr [rbp-0x21C]
       vmaxss   xmm11, xmm11, dword ptr [rbp-0x224]
       vmovss   dword ptr [rbp-0x22C], xmm11
       vmovss   xmm13, dword ptr [rbp-0x220]
       vminss   xmm9, xmm13, xmm9
       vmovss   dword ptr [rbp-0x230], xmm9
       vmovaps  xmm13, xmm1
       vbroadcastss xmm11, xmm11
       vmulps   xmm11, xmm11, xmm13
       vmovaps  xmm13, xmm1
       vbroadcastss xmm9, xmm9
       vmulps   xmm9, xmm9, xmm13
       vmovaps  xmm13, xmm5
       vaddps   xmm11, xmm11, xmm13
       vmovaps  xmm13, xmm5
       vaddps   xmm9, xmm9, xmm13
       vmovss   xmm13, dword ptr [rbp-0x1C4]
       vsubss   xmm13, xmm13, dword ptr [rbp-0x4A4]
       vmulss   xmm13, xmm13, dword ptr [reloc @RWD84]
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovaps  xmm13, xmmword ptr [rbp-0x4C0]
       vminss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm13
       vxorps   xmm13, xmm13, xmm13
       vmaxss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovss   xmm13, dword ptr [rbp-0x1C8]
       vsubss   xmm13, xmm13, dword ptr [rbp-0x4A4]
       vmulss   xmm13, xmm13, dword ptr [reloc @RWD84]
       vmovss   dword ptr [rbp-0x568], xmm13
       vmovaps  xmm13, xmmword ptr [rbp-0x4C0]
       vminss   xmm13, xmm13, dword ptr [rbp-0x568]
       vmovss   dword ptr [rbp-0x568], xmm13
       vxorps   xmm13, xmm13, xmm13
       vmaxss   xmm13, xmm13, dword ptr [rbp-0x568]
       vmulss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x234], xmm13
       vsubss   xmm13, xmm7, xmm13
       vmovss   dword ptr [rbp-0x238], xmm13
       vmovaps  xmm13, xmm0
       vsubps   xmm5, xmm13, xmm5
       vmovaps  xmm13, xmm1
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovaps  xmm13, xmm5
       vmulss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovshdup xmm1, xmm1
       vmovshdup xmm13, xmm5
       vmulss   xmm1, xmm1, xmm13
       vaddss   xmm1, xmm1, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x23C], xmm1
       vmovaps  xmm13, xmm3
       vmovaps  xmm1, xmm5
       vmulss   xmm1, xmm13, xmm1
       vmovshdup xmm3, xmm3
       vmovshdup xmm5, xmm5
       vmulss   xmm3, xmm3, xmm5
       vaddss   xmm1, xmm1, xmm3
 
G_M000_IG26:                ;; offset=0x1703
       vandps   xmm3, xmm1, xmmword ptr [reloc @RWD48]
       vmovss   xmm13, dword ptr [rbp-0x23C]
       vandps   xmm5, xmm13, xmmword ptr [reloc @RWD48]
       vucomiss xmm3, xmm5
       seta     dl
       movzx    rdx, dl
       vxorps   xmm3, xmm3, xmm3
       vucomiss xmm1, xmm3
       seta     r8b
       movzx    r8, r8b
       and      r8d, edx
       vucomiss xmm3, xmm1
       setae    r10b
       movzx    r10, r10b
       and      r10d, edx
       vucomiss xmm3, xmm13
       seta     r9b
       movzx    r9, r9b
       test     edx, edx
       sete     r11b
       movzx    r11, r11b
       and      r9d, r11d
       vucomiss xmm13, xmm3
       setae    r11b
       movzx    r11, r11b
       test     edx, edx
       sete     dl
       movzx    rdx, dl
       and      edx, r11d
       vmovaps  xmm1, xmm0
       vmovss   xmm13, dword ptr [rbp-0x238]
       vmulss   xmm1, xmm1, xmm13
       vmovaps  xmm3, xmm2
       vmovss   xmm5, dword ptr [rbp-0x234]
       vmulss   xmm3, xmm3, xmm5
       vaddss   xmm1, xmm1, xmm3
       vmovaps  xmm3, xmm2
       vmovss   dword ptr [rbp-0x30C], xmm3
       mov      r11d, -1
       xor      r14d, r14d
       test     r8d, r8d
       cmove    r11d, r14d
       vmovd    xmm3, r11d
       vandps   xmm1, xmm1, xmm3
       vmovups  xmmword ptr [rbp-0x590], xmm1
       vmovss   xmm1, dword ptr [rbp-0x30C]
       vandnps  xmm1, xmm3, xmm1
       vorps    xmm1, xmm1, xmmword ptr [rbp-0x590]
       vinsertps xmm2, xmm2, xmm1, 0
       vmovshdup xmm1, xmm0
       vmulss   xmm1, xmm1, xmm13
       vmovshdup xmm3, xmm2
       vmulss   xmm3, xmm3, xmm5
       vaddss   xmm1, xmm1, xmm3
       vmovshdup xmm3, xmm2
       vmovss   dword ptr [rbp-0x310], xmm3
       mov      r11d, -1
 
G_M000_IG27:                ;; offset=0x180B
       test     r8d, r8d
       cmove    r11d, r14d
       vmovd    xmm3, r11d
       vandps   xmm1, xmm1, xmm3
       vmovups  xmmword ptr [rbp-0x590], xmm1
       vmovss   xmm1, dword ptr [rbp-0x310]
       vandnps  xmm1, xmm3, xmm1
       vorps    xmm1, xmm1, xmmword ptr [rbp-0x590]
       vinsertps xmm2, xmm2, xmm1, 16
       vmovaps  xmm1, xmm0
       vmulss   xmm1, xmm1, xmm13
       vmovaps  xmm3, xmm4
       vmulss   xmm3, xmm3, xmm5
       vaddss   xmm1, xmm1, xmm3
       vmovaps  xmm3, xmm4
       vmovss   dword ptr [rbp-0x314], xmm3
       mov      r8d, -1
 
G_M000_IG28:                ;; offset=0x1864
       test     r10d, r10d
       cmove    r8d, r14d
       vmovd    xmm3, r8d
       vandps   xmm1, xmm1, xmm3
       vmovups  xmmword ptr [rbp-0x590], xmm1
       vmovss   xmm1, dword ptr [rbp-0x314]
       vandnps  xmm1, xmm3, xmm1
       vorps    xmm1, xmm1, xmmword ptr [rbp-0x590]
       vinsertps xmm4, xmm4, xmm1, 0
       vmovshdup xmm1, xmm0
       vmulss   xmm1, xmm1, xmm13
       vmovshdup xmm3, xmm4
       vmulss   xmm3, xmm3, xmm5
       vaddss   xmm1, xmm1, xmm3
       vmovshdup xmm3, xmm4
       vmovss   dword ptr [rbp-0x318], xmm3
       mov      r8d, -1
 
G_M000_IG29:                ;; offset=0x18BD
       test     r10d, r10d
       cmove    r8d, r14d
       vmovd    xmm3, r8d
       vandps   xmm1, xmm1, xmm3
       vmovups  xmmword ptr [rbp-0x590], xmm1
       vmovss   xmm1, dword ptr [rbp-0x318]
       vandnps  xmm1, xmm3, xmm1
       vorps    xmm1, xmm1, xmmword ptr [rbp-0x590]
       vinsertps xmm4, xmm4, xmm1, 16
       vmovaps  xmm1, xmm0
       vmulss   xmm1, xmm1, xmm13
       vmovaps  xmm3, xmm11
       vmulss   xmm3, xmm3, xmm5
       vaddss   xmm1, xmm1, xmm3
       vmovaps  xmm3, xmm11
       vmovss   dword ptr [rbp-0x31C], xmm3
       mov      r8d, -1
 
G_M000_IG30:                ;; offset=0x1918
       test     r9d, r9d
       cmove    r8d, r14d
       vmovd    xmm3, r8d
       vandps   xmm1, xmm1, xmm3
       vmovups  xmmword ptr [rbp-0x590], xmm1
       vmovss   xmm1, dword ptr [rbp-0x31C]
       vandnps  xmm1, xmm3, xmm1
       vorps    xmm1, xmm1, xmmword ptr [rbp-0x590]
       vinsertps xmm11, xmm11, xmm1, 0
       vmovshdup xmm1, xmm0
       vmulss   xmm1, xmm1, xmm13
       vmovshdup xmm3, xmm11
       vmulss   xmm3, xmm3, xmm5
       vaddss   xmm1, xmm1, xmm3
       vmovshdup xmm3, xmm11
       vmovss   dword ptr [rbp-0x320], xmm3
       mov      r8d, -1
 
G_M000_IG31:                ;; offset=0x1973
       test     r9d, r9d
       cmove    r8d, r14d
       vmovd    xmm3, r8d
       vandps   xmm1, xmm1, xmm3
       vmovups  xmmword ptr [rbp-0x590], xmm1
       vmovss   xmm1, dword ptr [rbp-0x320]
       vandnps  xmm1, xmm3, xmm1
       vorps    xmm1, xmm1, xmmword ptr [rbp-0x590]
       vinsertps xmm11, xmm11, xmm1, 16
       vmovaps  xmm1, xmm0
       vmulss   xmm1, xmm1, xmm13
       vmovaps  xmm3, xmm9
       vmulss   xmm3, xmm3, xmm5
       vaddss   xmm1, xmm1, xmm3
       vmovaps  xmm3, xmm9
       vmovss   dword ptr [rbp-0x324], xmm3
       mov      r8d, -1
 
G_M000_IG32:                ;; offset=0x19CE
       test     edx, edx
       cmove    r8d, r14d
       vmovd    xmm3, r8d
       vandps   xmm1, xmm1, xmm3
       vmovups  xmmword ptr [rbp-0x590], xmm1
       vmovss   xmm1, dword ptr [rbp-0x324]
       vandnps  xmm1, xmm3, xmm1
       vorps    xmm1, xmm1, xmmword ptr [rbp-0x590]
       vinsertps xmm9, xmm9, xmm1, 0
       vmovshdup xmm1, xmm0
       vmulss   xmm1, xmm1, xmm13
       vmovshdup xmm3, xmm9
       vmulss   xmm3, xmm3, xmm5
       vaddss   xmm1, xmm1, xmm3
       vmovshdup xmm3, xmm9
       mov      r8d, -1
 
G_M000_IG33:                ;; offset=0x1A20
       test     edx, edx
       cmove    r8d, r14d
       vmovd    xmm5, r8d
       vandps   xmm1, xmm1, xmm5
       vandnps  xmm3, xmm5, xmm3
       vorps    xmm1, xmm3, xmm1
       vinsertps xmm9, xmm9, xmm1, 16
       vmovaps  xmm1, xmm4
       vinsertps xmm1, xmm15, xmm1, 0
       vmovaps  xmm3, xmm12
       vinsertps xmm1, xmm1, xmm3, 16
       vmovshdup xmm3, xmm4
       vinsertps xmm5, xmm1, xmm3, 32
       vmovaps  xmm1, xmm11
       vinsertps xmm1, xmm15, xmm1, 0
       vmovaps  xmm3, xmm12
       vinsertps xmm1, xmm1, xmm3, 16
       vmovshdup xmm3, xmm11
       vinsertps xmm1, xmm1, xmm3, 32
       vmovaps  xmmword ptr [rbp-0x170], xmm1
       vmovaps  xmm3, xmm9
       vinsertps xmm3, xmm15, xmm3, 0
       vmovaps  xmm4, xmm12
       vinsertps xmm3, xmm3, xmm4, 16
       vmovshdup xmm4, xmm9
       vinsertps xmm13, xmm3, xmm4, 32
       vmovss   xmm4, dword ptr [rbp-0x200]
       vucomiss xmm4, dword ptr [rbp-0x1FC]
       seta     dl
       mov      byte  ptr [rbx+0x5D], dl
       movzx    rdx, byte  ptr [rbx+0x5D]
       mov      byte  ptr [rbx+0x5E], dl
       movzx    rdx, byte  ptr [rbx+0x5D]
       vmovss   xmm9, dword ptr [rbp-0x230]
       vucomiss xmm9, dword ptr [rbp-0x22C]
       seta     r8b
       movzx    r8, r8b
       and      edx, r8d
       mov      byte  ptr [rbx+0x5F], dl
       vmovsd   qword ptr [rbp-0x180], xmm13
       vextractps dword ptr [rbp-0x178], xmm13, 2
       vmovaps  xmm3, xmm2
       vmovss   xmm4, dword ptr [rbp-0x144]
 
G_M000_IG34:                ;; offset=0x1B03
       vmovaps  xmm2, xmm3
       vinsertps xmm2, xmm15, xmm2, 0
       vmovaps  xmm15, xmm12
       vinsertps xmm2, xmm2, xmm15, 16
       vmovshdup xmm3, xmm3
       vinsertps xmm15, xmm2, xmm3, 32
       mov      byte  ptr [rbx+0x5C], 1
 
G_M000_IG35:                ;; offset=0x1B26
       vmovups  xmm2, xmmword ptr [rbp-0xE8]
       vinsertps xmm2, xmm2, xmm2, 56
       vmovaps  xmm1, xmmword ptr [rbp-0x120]
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm1, xmm1, xmm2
       vpermilps xmm2, xmm1, -11
       vaddps   xmm2, xmm2, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm2
       vmovaps  xmm2, xmmword ptr [rbp-0xD0]
       vinsertps xmm2, xmm2, xmm2, 56
       vmovaps  xmm4, xmmword ptr [rbp-0x120]
       vinsertps xmm4, xmm4, xmm4, 56
       vmulps   xmm2, xmm4, xmm2
       vpermilps xmm4, xmm2, -11
       vaddps   xmm4, xmm4, xmm2
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm4
       vmulss   xmm4, xmm1, xmm1
       vmulss   xmm9, xmm2, xmm2
       vaddss   xmm4, xmm4, xmm9
       vsqrtss  xmm4, xmm4, xmm4
       vdivss   xmm4, xmm7, xmm4
       vmulss   xmm1, xmm1, xmm4
       vbroadcastss xmm1, xmm1
       vmovups  xmm9, xmmword ptr [rbp-0xE8]
       vmulps   xmm1, xmm1, xmm9
       vmulss   xmm2, xmm2, xmm4
       vbroadcastss xmm2, xmm2
       vmovaps  xmm4, xmmword ptr [rbp-0xD0]
       vmulps   xmm2, xmm2, xmm4
       vaddps   xmm1, xmm2, xmm1
       vmovaps  xmmword ptr [rbp-0x1C0], xmm1
       vxorps   xmm2, xmm2, xmm2
       vmovaps  xmm11, xmmword ptr [rbp-0x190]
       vmovaps  xmm4, xmm11
       vaddss   xmm4, xmm4, dword ptr [rbp-0x110]
       vinsertps xmm2, xmm2, xmm4, 14
       vmovshdup xmm4, xmm11
       vaddss   xmm4, xmm4, dword ptr [rbp-0x10C]
       vinsertps xmm2, xmm2, xmm4, 16
       vunpckhps xmm4, xmm11, xmm11
       vaddss   xmm4, xmm4, dword ptr [rbp-0x108]
       vinsertps xmm2, xmm2, xmm4, 32
       vxorps   xmm4, xmm4, xmm4
       vmovaps  xmm9, xmm0
       vinsertps xmm4, xmm4, xmm9, 14
       vmovshdup xmm9, xmm0
       vinsertps xmm4, xmm4, xmm9, 32
       cmp      eax, ecx
       je       G_M000_IG53
       vmovaps  xmm13, xmm2
       vmovups  xmm14, xmmword ptr [rbp-0xDC]
       vaddps   xmm9, xmm14, xmm13
       vxorps   xmm11, xmm11, xmm11
       vmovaps  xmm13, xmm4
       vinsertps xmm11, xmm11, xmm13, 14
       vmovaps  xmm13, xmm7
 
G_M000_IG36:                ;; offset=0x1C63
       vinsertps xmm11, xmm11, xmm13, 16
       vunpckhps xmm13, xmm4, xmm4
       vinsertps xmm11, xmm11, xmm13, 32
       vmovaps  xmmword ptr [rbp-0x250], xmm11
       vmovaps  xmm14, xmmword ptr [rbp-0x160]
       vmovaps  xmm13, xmm4
       vsubps   xmm13, xmm14, xmm13
       vinsertps xmm13, xmm13, xmm13, 56
       vmovups  xmm1, xmmword ptr [rbp-0xDC]
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm1, xmm1, xmm13
       vpermilps xmm13, xmm1, -11
       vaddps   xmm13, xmm13, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm13
       vmovss   xmm13, dword ptr [rbp-0x144]
       vmulss   xmm1, xmm1, xmm13
       vbroadcastss xmm1, xmm1
       vmovaps  xmm11, xmmword ptr [rbp-0x120]
       vmulps   xmm1, xmm1, xmm11
       vmovaps  xmm11, xmm4
       vaddps   xmm1, xmm1, xmm11
       vsubps   xmm1, xmm1, xmm14
       vmovaps  xmmword ptr [rbp-0x340], xmm1
       vmovaps  xmm11, xmm1
       vinsertps xmm11, xmm11, xmm11, 56
       vmovups  xmm1, xmmword ptr [rbp-0xE8]
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm1, xmm1, xmm11
       vpermilps xmm11, xmm1, -11
       vaddps   xmm11, xmm11, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm11
       vmovss   dword ptr [rbp-0x378], xmm1
       vmovaps  xmm11, xmmword ptr [rbp-0x340]
       vinsertps xmm11, xmm11, xmm11, 56
       vmovaps  xmm1, xmmword ptr [rbp-0xD0]
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm1, xmm1, xmm11
       vpermilps xmm11, xmm1, -11
       vaddps   xmm11, xmm11, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm11
       vmovss   dword ptr [rbp-0x37C], xmm1
       vmovshdup xmm11, xmm2
       vsubss   xmm11, xmm11, xmm12
       vmulss   xmm11, xmm11, dword ptr [rbp-0x148]
       vmovss   dword ptr [rbp-0x344], xmm11
       vmulss   xmm1, xmm11, dword ptr [rbp-0x120]
       vmovaps  xmm11, xmm2
       vsubss   xmm1, xmm11, xmm1
       vmovss   dword ptr [rbp-0x380], xmm1
       vmovss   xmm11, dword ptr [rbp-0x344]
       vmulss   xmm11, xmm11, dword ptr [rbp-0x118]
       vunpckhps xmm1, xmm2, xmm2
       vsubss   xmm1, xmm1, xmm11
       vmovss   dword ptr [rbp-0x384], xmm1
       vmovaps  xmm11, xmmword ptr [rbp-0x250]
 
G_M000_IG37:                ;; offset=0x1DC1
       vsubps   xmm11, xmm14, xmm11
       vinsertps xmm11, xmm11, xmm11, 56
       vmovups  xmm1, xmmword ptr [rbp-0xDC]
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm1, xmm1, xmm11
       vpermilps xmm11, xmm1, -11
       vaddps   xmm11, xmm11, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm11
       vmulss   xmm1, xmm1, xmm13
       vbroadcastss xmm1, xmm1
       vmovaps  xmm11, xmmword ptr [rbp-0x120]
       vmulps   xmm1, xmm1, xmm11
       vmovaps  xmm11, xmmword ptr [rbp-0x250]
       vaddps   xmm1, xmm1, xmm11
       vsubps   xmm1, xmm1, xmm14
       vmovaps  xmm11, xmm1
       vinsertps xmm11, xmm11, xmm11, 56
       vmovups  xmm13, xmmword ptr [rbp-0xE8]
       vinsertps xmm13, xmm13, xmm13, 56
       vmulps   xmm11, xmm13, xmm11
       vpermilps xmm13, xmm11, -11
       vaddps   xmm13, xmm13, xmm11
       vpermilps xmm11, xmm11, -86
       vaddps   xmm11, xmm11, xmm13
       vmovss   dword ptr [rbp-0x388], xmm11
       vinsertps xmm1, xmm1, xmm1, 56
       vmovaps  xmm13, xmmword ptr [rbp-0xD0]
       vinsertps xmm13, xmm13, xmm13, 56
       vmulps   xmm1, xmm13, xmm1
       vpermilps xmm13, xmm1, -11
       vaddps   xmm13, xmm13, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm13
       vmovss   dword ptr [rbp-0x38C], xmm1
       vmovshdup xmm13, xmm9
       vsubss   xmm13, xmm13, xmm12
       vmulss   xmm13, xmm13, dword ptr [rbp-0x148]
       vmulss   xmm1, xmm13, dword ptr [rbp-0x120]
       vmovaps  xmm11, xmm9
       vsubss   xmm1, xmm11, xmm1
       vmovss   dword ptr [rbp-0x390], xmm1
       vmulss   xmm11, xmm13, dword ptr [rbp-0x118]
       vunpckhps xmm9, xmm9, xmm9
       vsubss   xmm9, xmm9, xmm11
       vmovss   dword ptr [rbp-0x394], xmm9
       mov      edx, -1
       xor      r8d, r8d
       test     eax, eax
       cmove    edx, r8d
       vxorps   xmm11, xmm11, xmm11
       vmovd    xmm13, edx
       vmovss   xmm9, dword ptr [rbp-0x378]
       vandps   xmm9, xmm9, xmm13
       vmovss   xmm1, dword ptr [rbp-0x380]
       vandnps  xmm1, xmm13, xmm1
       vorps    xmm1, xmm1, xmm9
       vinsertps xmm1, xmm11, xmm1, 14
       mov      edx, -1
 
G_M000_IG38:                ;; offset=0x1F12
       test     eax, eax
       cmove    edx, r8d
       vmovd    xmm9, edx
       vmovss   xmm11, dword ptr [rbp-0x37C]
       vandps   xmm11, xmm11, xmm9
       vmovss   xmm13, dword ptr [rbp-0x384]
       vandnps  xmm9, xmm9, xmm13
       vorps    xmm9, xmm9, xmm11
       vinsertps xmm1, xmm1, xmm9, 16
       mov      edx, -1
 
G_M000_IG39:                ;; offset=0x1F46
       test     eax, eax
       cmove    edx, r8d
       vxorps   xmm9, xmm9, xmm9
       vmovsd   qword ptr [rbp-0x570], xmm9
       vmovd    xmm11, edx
       vmovss   xmm13, dword ptr [rbp-0x388]
       vandps   xmm13, xmm13, xmm11
       vmovss   xmm9, dword ptr [rbp-0x390]
       vandnps  xmm9, xmm11, xmm9
       vorps    xmm9, xmm9, xmm13
       vmovsd   xmm11, qword ptr [rbp-0x570]
       vinsertps xmm9, xmm11, xmm9, 14
       vmovsd   qword ptr [rbp-0x258], xmm9
       mov      edx, -1
 
G_M000_IG40:                ;; offset=0x1F97
       test     eax, eax
       cmove    edx, r8d
       vmovd    xmm11, edx
       vmovss   xmm13, dword ptr [rbp-0x38C]
       vandps   xmm13, xmm13, xmm11
       vmovss   xmm9, dword ptr [rbp-0x394]
       vandnps  xmm9, xmm11, xmm9
       vorps    xmm9, xmm9, xmm13
       vmovsd   xmm11, qword ptr [rbp-0x258]
       vinsertps xmm11, xmm11, xmm9, 16
       vmovsd   qword ptr [rbp-0x258], xmm11
       vmovss   xmm9, dword ptr [rsi]
       vmovss   xmm13, dword ptr [rdi]
       mov      edx, -1
 
G_M000_IG41:                ;; offset=0x1FE3
       test     eax, eax
       cmove    edx, r8d
       vmovd    xmm11, edx
       vandps   xmm9, xmm9, xmm11
       vandnps  xmm11, xmm11, xmm13
       vorps    xmm9, xmm11, xmm9
       vmovss   dword ptr [rbp-0x25C], xmm9
       vmovss   xmm11, dword ptr [rdi+0x04]
       vmovss   xmm13, dword ptr [rsi+0x04]
       mov      edx, -1
 
G_M000_IG42:                ;; offset=0x2013
       test     eax, eax
       cmove    edx, r8d
       vmovd    xmm9, edx
       vandps   xmm11, xmm11, xmm9
       vandnps  xmm9, xmm9, xmm13
       vorps    xmm9, xmm9, xmm11
       vmovss   dword ptr [rbp-0x264], xmm9
       vmovsd   xmm11, qword ptr [rbp-0x258]
       vmovaps  xmm13, xmm1
       vsubps   xmm11, xmm11, xmm13
       vmovaps  xmm13, xmm11
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovaps  xmm13, xmm11
       vmulss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovshdup xmm13, xmm11
       vmovss   dword ptr [rbp-0x568], xmm13
       vmovshdup xmm13, xmm11
       vmulss   xmm13, xmm13, dword ptr [rbp-0x568]
       vaddss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x348], xmm13
       vdivss   xmm13, xmm7, xmm13
       vmovss   dword ptr [rbp-0x34C], xmm13
       vmovaps  xmm13, xmm1
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovaps  xmm13, xmm11
       vmulss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovshdup xmm13, xmm1
       vmovss   dword ptr [rbp-0x568], xmm13
       vmovshdup xmm13, xmm11
       vmulss   xmm13, xmm13, dword ptr [rbp-0x568]
       vaddss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x350], xmm13
       vmovaps  xmm13, xmm1
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovaps  xmm13, xmm1
       vmulss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovshdup xmm13, xmm1
       vmovss   dword ptr [rbp-0x568], xmm13
       vmovshdup xmm13, xmm1
       vmulss   xmm13, xmm13, dword ptr [rbp-0x568]
       vaddss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovss   xmm13, dword ptr [rbp-0x25C]
       vmulss   xmm13, xmm13, xmm13
       vmovss   dword ptr [rbp-0x568], xmm13
       vmovss   xmm13, dword ptr [rbp-0x564]
       vsubss   xmm13, xmm13, dword ptr [rbp-0x568]
       vmovss   dword ptr [rbp-0x354], xmm13
       vmovss   xmm13, dword ptr [rbp-0x350]
       vmulss   xmm13, xmm13, xmm13
       vmovss   dword ptr [rbp-0x568], xmm13
       vmovss   xmm13, dword ptr [rbp-0x348]
       vmulss   xmm13, xmm13, dword ptr [rbp-0x354]
       vmovss   dword ptr [rbp-0x564], xmm13
       vmovss   xmm13, dword ptr [rbp-0x568]
       vsubss   xmm13, xmm13, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x564], xmm13
       vxorps   xmm13, xmm13, xmm13
       vmaxss   xmm13, xmm13, dword ptr [rbp-0x564]
       vsqrtss  xmm13, xmm13, xmm13
       vmulss   xmm13, xmm13, dword ptr [rbp-0x34C]
 
G_M000_IG43:                ;; offset=0x21BC
       vmovss   dword ptr [rbp-0x358], xmm13
       vmovss   xmm3, dword ptr [rbp-0x350]
       vxorps   xmm3, xmm3, xmmword ptr [reloc @RWD16]
       vmulss   xmm3, xmm3, dword ptr [rbp-0x34C]
       vmovss   dword ptr [rbp-0x35C], xmm3
       vsubss   xmm13, xmm3, dword ptr [rbp-0x358]
       vaddss   xmm3, xmm3, dword ptr [rbp-0x358]
       vmovss   dword ptr [rbp-0x260], xmm3
       vmovss   xmm3, dword ptr [rbp-0x348]
       vandps   xmm3, xmm3, xmmword ptr [reloc @RWD48]
       vmovss   xmm9, dword ptr [reloc @RWD76]
       vucomiss xmm9, xmm3
       seta     dl
       movzx    rdx, dl
       mov      r8d, -1
       xor      r10d, r10d
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm3, r8d
       vmovss   xmm9, dword ptr [rbp-0x35C]
       vandps   xmm9, xmm9, xmm3
       vandnps  xmm3, xmm3, xmm13
       vorps    xmm13, xmm3, xmm9
       mov      r8d, -1
 
G_M000_IG44:                ;; offset=0x224E
       test     edx, edx
       cmove    r8d, r10d
       vmovd    xmm3, r8d
       vmovss   xmm9, dword ptr [rbp-0x35C]
       vandps   xmm9, xmm9, xmm3
       vmovups  xmmword ptr [rbp-0x590], xmm9
       vmovss   xmm9, dword ptr [rbp-0x260]
       vandnps  xmm3, xmm3, xmm9
       vorps    xmm3, xmm3, xmmword ptr [rbp-0x590]
       vmovss   dword ptr [rbp-0x260], xmm3
       vmovss   xmm3, dword ptr [rbp-0x264]
       vxorps   xmm9, xmm3, xmmword ptr [reloc @RWD16]
       vmaxss   xmm13, xmm9, xmm13
       vmovaps  xmm9, xmm3
       vminss   xmm13, xmm9, xmm13
       vminss   xmm3, xmm3, dword ptr [rbp-0x260]
       vmovaps  xmm9, xmm4
       vmovss   dword ptr [rbp-0x360], xmm9
       vmovaps  xmm9, xmm11
       vmulss   xmm9, xmm13, xmm9
       vmovss   dword ptr [rbp-0x564], xmm9
       vmovaps  xmm9, xmm1
       vaddss   xmm9, xmm9, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x364], xmm9
       mov      edx, -1
 
G_M000_IG45:                ;; offset=0x22E7
       test     eax, eax
       cmove    edx, r10d
       vmovd    xmm9, edx
       vmovaps  xmmword ptr [rbp-0x520], xmm9
       vmovss   xmm9, dword ptr [rbp-0x360]
       vandps   xmm9, xmm9, xmmword ptr [rbp-0x520]
       vmovups  xmmword ptr [rbp-0x590], xmm9
       vmovss   xmm9, dword ptr [rbp-0x364]
       vmovups  xmmword ptr [rbp-0x580], xmm9
       vmovaps  xmm9, xmmword ptr [rbp-0x520]
       vandnps  xmm9, xmm9, xmmword ptr [rbp-0x580]
       vorps    xmm9, xmm9, xmmword ptr [rbp-0x590]
       vinsertps xmm15, xmm15, xmm9, 0
       mov      edx, -1
 
G_M000_IG46:                ;; offset=0x2344
       test     eax, eax
       cmove    edx, r10d
       vmovd    xmm9, edx
       vmovaps  xmmword ptr [rbp-0x530], xmm9
       vmovaps  xmm9, xmm13
       vandps   xmm9, xmm9, xmmword ptr [rbp-0x530]
       vmovups  xmmword ptr [rbp-0x590], xmm9
       vmovaps  xmm9, xmm12
       vmovups  xmmword ptr [rbp-0x580], xmm9
       vmovaps  xmm9, xmmword ptr [rbp-0x530]
       vandnps  xmm9, xmm9, xmmword ptr [rbp-0x580]
       vorps    xmm9, xmm9, xmmword ptr [rbp-0x590]
       vinsertps xmm15, xmm15, xmm9, 16
       vunpckhps xmm9, xmm4, xmm4
       vmovss   dword ptr [rbp-0x368], xmm9
       vmovshdup xmm9, xmm11
       vmulss   xmm9, xmm13, xmm9
       vmovss   dword ptr [rbp-0x564], xmm9
       vmovshdup xmm9, xmm1
       vaddss   xmm9, xmm9, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x36C], xmm9
       mov      edx, -1
 
G_M000_IG47:                ;; offset=0x23CD
       test     eax, eax
       cmove    edx, r10d
       vmovd    xmm9, edx
       vmovaps  xmmword ptr [rbp-0x540], xmm9
       vmovss   xmm9, dword ptr [rbp-0x368]
       vandps   xmm9, xmm9, xmmword ptr [rbp-0x540]
       vmovups  xmmword ptr [rbp-0x590], xmm9
       vmovss   xmm9, dword ptr [rbp-0x36C]
       vmovups  xmmword ptr [rbp-0x580], xmm9
       vmovaps  xmm9, xmmword ptr [rbp-0x540]
       vandnps  xmm9, xmm9, xmmword ptr [rbp-0x580]
       vorps    xmm9, xmm9, xmmword ptr [rbp-0x590]
       vinsertps xmm15, xmm15, xmm9, 32
       vmovaps  xmm9, xmm4
       vmovss   dword ptr [rbp-0x370], xmm9
       vmovaps  xmm9, xmm11
       vmulss   xmm9, xmm3, xmm9
       vmovss   dword ptr [rbp-0x564], xmm9
       vmovaps  xmm9, xmm1
       vaddss   xmm9, xmm9, dword ptr [rbp-0x564]
       vmovss   dword ptr [rbp-0x374], xmm9
       mov      edx, -1
 
G_M000_IG48:                ;; offset=0x245C
       test     eax, eax
       cmove    edx, r10d
       vmovd    xmm9, edx
       vmovaps  xmmword ptr [rbp-0x550], xmm9
       vmovss   xmm9, dword ptr [rbp-0x370]
       vandps   xmm9, xmm9, xmmword ptr [rbp-0x550]
       vmovups  xmmword ptr [rbp-0x590], xmm9
       vmovss   xmm9, dword ptr [rbp-0x374]
       vmovups  xmmword ptr [rbp-0x580], xmm9
       vmovaps  xmm9, xmmword ptr [rbp-0x550]
       vandnps  xmm9, xmm9, xmmword ptr [rbp-0x580]
       vorps    xmm9, xmm9, xmmword ptr [rbp-0x590]
       vinsertps xmm5, xmm5, xmm9, 0
       mov      edx, -1
 
G_M000_IG49:                ;; offset=0x24B9
       test     eax, eax
       cmove    edx, r10d
       vmovd    xmm9, edx
       vmovaps  xmmword ptr [rbp-0x560], xmm9
       vmovaps  xmm9, xmm3
       vandps   xmm9, xmm9, xmmword ptr [rbp-0x560]
       vmovups  xmmword ptr [rbp-0x590], xmm9
       vmovaps  xmm9, xmmword ptr [rbp-0x560]
       vandnps  xmm9, xmm9, xmm12
       vorps    xmm9, xmm9, xmmword ptr [rbp-0x590]
       vinsertps xmm5, xmm5, xmm9, 16
       vunpckhps xmm4, xmm4, xmm4
       vmovshdup xmm9, xmm11
       vmulss   xmm9, xmm3, xmm9
       vmovshdup xmm1, xmm1
       vaddss   xmm1, xmm9, xmm1
       mov      edx, -1
 
G_M000_IG50:                ;; offset=0x2515
       test     eax, eax
       cmove    edx, r10d
       vmovd    xmm9, edx
       vandps   xmm4, xmm4, xmm9
       vandnps  xmm1, xmm9, xmm1
       vorps    xmm1, xmm1, xmm4
       vinsertps xmm5, xmm5, xmm1, 32
       mov      byte  ptr [rbx+0x5C], 1
       vucomiss xmm3, xmm13
       seta     dl
       mov      byte  ptr [rbx+0x5D], dl
       mov      edx, -1
 
G_M000_IG51:                ;; offset=0x2546
       test     eax, eax
       cmove    edx, r10d
       vmovd    xmm1, edx
       vpbroadcastd xmm1, xmm1
       vmovaps  xmm3, xmmword ptr [rbp-0x1A0]
       vandps   xmm3, xmm3, xmm1
       vmovaps  xmm13, xmmword ptr [rbp-0x1C0]
       vmovaps  xmm4, xmm13
       vandnps  xmm1, xmm1, xmm4
       vorps    xmm1, xmm1, xmm3
       mov      edx, -1
 
G_M000_IG52:                ;; offset=0x257B
       test     eax, eax
       cmove    edx, r10d
       vmovd    xmm3, edx
       vpbroadcastd xmm3, xmm3
       vandps   xmm4, xmm14, xmm3
       vmovaps  xmm9, xmm2
       vandnps  xmm3, xmm3, xmm9
       vorps    xmm14, xmm3, xmm4
       vmovsd   qword ptr [rbp-0x1B0], xmm1
       vextractps dword ptr [rbp-0x1A8], xmm1, 2
 
G_M000_IG53:                ;; offset=0x25AD
       test     eax, eax
       sete     al
       movzx    rax, al
       test     ecx, ecx
       sete     cl
       movzx    rcx, cl
       test     eax, ecx
       je       G_M000_IG55
       vmovaps  xmm3, xmm2
       vxorps   xmm3, xmm3, xmmword ptr [reloc @RWD16]
       vmovss   xmm4, dword ptr [rbp-0x120]
       vmulss   xmm4, xmm4, dword ptr [rbp-0x120]
       vmovss   xmm13, dword ptr [rbp-0x118]
       vmulss   xmm14, xmm13, dword ptr [rbp-0x118]
       vaddss   xmm4, xmm4, xmm14
       vdivss   xmm4, xmm7, xmm4
       vmovss   xmm9, dword ptr [rsi+0x04]
       vmovss   xmm11, dword ptr [rdi+0x04]
       vmovups  xmm12, xmmword ptr [rbp-0xDC]
       vinsertps xmm12, xmm12, xmm12, 56
       vmovaps  xmm13, xmm3
       vinsertps xmm13, xmm13, xmm13, 56
       vmulps   xmm12, xmm13, xmm12
       vpermilps xmm13, xmm12, -11
       vaddps   xmm13, xmm13, xmm12
       vpermilps xmm12, xmm12, -86
       vaddps   xmm12, xmm12, xmm13
       vmovshdup xmm3, xmm3
       vmovss   xmm13, dword ptr [rbp-0xD8]
       vmulss   xmm14, xmm3, xmm13
       vsubss   xmm12, xmm12, xmm14
       vmulss   xmm14, xmm13, xmm13
       vsubss   xmm14, xmm7, xmm14
       vbroadcastss xmm1, dword ptr [reloc @RWD88]
       vmaxss   xmm1, xmm1, xmm14
       vdivss   xmm1, xmm12, xmm1
       vmulss   xmm1, xmm1, xmm13
       vsubss   xmm1, xmm1, xmm3
       vandps   xmm13, xmm13, xmmword ptr [reloc @RWD48]
       vmulss   xmm9, xmm13, xmm9
       vxorps   xmm12, xmm11, xmmword ptr [reloc @RWD16]
       vxorps   xmm13, xmm9, xmmword ptr [reloc @RWD16]
       vsubss   xmm13, xmm13, xmm3
       vmovaps  xmm14, xmm11
       vminss   xmm13, xmm14, xmm13
       vmaxss   xmm12, xmm12, xmm13
       vxorps   xmm13, xmm11, xmmword ptr [reloc @RWD16]
       vsubss   xmm3, xmm9, xmm3
       vmaxss   xmm3, xmm13, xmm3
       vminss   xmm3, xmm11, xmm3
       vmaxss   xmm1, xmm1, xmm12
       vminss   xmm1, xmm1, xmm3
       vmovss   xmm9, dword ptr [rbp-0xDC]
       vmulss   xmm9, xmm9, dword ptr [rbp-0x118]
       vmovss   xmm11, dword ptr [rbp-0xD4]
       vmulss   xmm11, xmm11, dword ptr [rbp-0x120]
       vsubss   xmm9, xmm9, xmm11
       vmulss   xmm9, xmm9, xmm9
       vmulss   xmm4, xmm9, xmm4
       vmovss   xmm9, dword ptr [reloc @RWD92]
 
G_M000_IG54:                ;; offset=0x26F5
       vsubss   xmm4, xmm9, xmm4
       vmulss   xmm4, xmm4, dword ptr [reloc @RWD96]
       vbroadcastss xmm13, dword ptr [reloc @RWD00]
       vminss   xmm4, xmm13, xmm4
       vxorps   xmm9, xmm9, xmm9
       vmaxss   xmm4, xmm9, xmm4
       vmulss   xmm9, xmm1, xmm4
       vsubss   xmm1, xmm1, xmm9
       vmulss   xmm9, xmm4, xmm12
       vaddss   xmm9, xmm9, xmm1
       vmulss   xmm3, xmm4, xmm3
       vaddss   xmm1, xmm3, xmm1
       vmovaps  xmm3, xmm0
       vinsertps xmm3, xmm15, xmm3, 0
       vmovaps  xmm4, xmm9
       vinsertps xmm3, xmm3, xmm4, 16
       vmovshdup xmm4, xmm0
       vinsertps xmm15, xmm3, xmm4, 32
       vmovaps  xmm3, xmm0
       vinsertps xmm5, xmm5, xmm3, 0
       vmovaps  xmm3, xmm1
       vinsertps xmm3, xmm5, xmm3, 16
       vmovshdup xmm0, xmm0
       vinsertps xmm5, xmm3, xmm0, 32
       mov      byte  ptr [rbx+0x5C], 1
       vucomiss xmm1, xmm9
       seta     al
       mov      byte  ptr [rbx+0x5D], al
       vmovaps  xmm1, xmmword ptr [rbp-0x1C0]
       vmovaps  xmm14, xmm2
       vmovsd   qword ptr [rbp-0x1B0], xmm1
       vextractps dword ptr [rbp-0x1A8], xmm1, 2
 
G_M000_IG55:                ;; offset=0x279B
       vmovaps  xmm13, xmmword ptr [rbp-0x1B0]
       vmovaps  xmm0, xmm13
       vinsertps xmm0, xmm0, xmm0, 56
       vmovaps  xmm1, xmmword ptr [rbp-0x120]
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm0, xmm1, xmm0
       vpermilps xmm1, xmm0, -11
       vaddps   xmm1, xmm1, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm1
       vdivss   xmm0, xmm7, xmm0
       vmovss   xmm2, dword ptr [rbp+0x20]
       vxorps   xmm1, xmm2, xmmword ptr [reloc @RWD16]
       lea      rax, bword ptr [rbx+0x3C]
       lea      rcx, bword ptr [rbx+0x5C]
       vmovaps  xmm2, xmm15
       vmovaps  xmm3, xmm14
       vsubps   xmm2, xmm2, xmm3
       vinsertps xmm2, xmm2, xmm2, 56
       vmovaps  xmm3, xmm13
       vinsertps xmm3, xmm3, xmm3, 56
       vmulps   xmm2, xmm3, xmm2
       vpermilps xmm3, xmm2, -11
       vaddps   xmm3, xmm3, xmm2
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm3
       vmulss   xmm2, xmm2, xmm0
       vmovss   dword ptr [rax], xmm2
       vmovaps  xmm9, xmmword ptr [rbp-0x100]
       vmovaps  xmm2, xmm9
       vaddps   xmm2, xmm2, xmm15
       vmovaps  xmm3, xmm10
       vmovaps  xmm4, xmm2
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm6
       vmovshdup xmm7, xmm2
       vbroadcastss xmm7, xmm7
       vmulps   xmm4, xmm7, xmm4
       vaddps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm8
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
       vmovaps  xmm2, xmm5
       vmovaps  xmm3, xmm14
       vsubps   xmm2, xmm2, xmm3
       vinsertps xmm2, xmm2, xmm2, 56
       vmovaps  xmm3, xmm13
       vinsertps xmm3, xmm3, xmm3, 56
       vmulps   xmm2, xmm3, xmm2
       vpermilps xmm3, xmm2, -11
 
G_M000_IG56:                ;; offset=0x28CA
       vaddps   xmm3, xmm3, xmm2
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm3
       vmulss   xmm2, xmm2, xmm0
       vmovss   dword ptr [rcx], xmm2
       vmovaps  xmm2, xmm9
       vaddps   xmm2, xmm2, xmm5
       vmovaps  xmm3, xmm10
       vmovaps  xmm4, xmm2
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm6
       vmovshdup xmm5, xmm2
       vbroadcastss xmm5, xmm5
       vmulps   xmm4, xmm5, xmm4
       vaddps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm8
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
       vmovaps  xmm2, xmmword ptr [rbp-0x170]
       vmovaps  xmm3, xmm2
       vmovaps  xmm4, xmm14
       vsubps   xmm3, xmm3, xmm4
       vinsertps xmm3, xmm3, xmm3, 56
       vmovaps  xmm4, xmm13
       vinsertps xmm4, xmm4, xmm4, 56
       vmulps   xmm3, xmm4, xmm3
       vpermilps xmm4, xmm3, -11
       vaddps   xmm4, xmm4, xmm3
       vpermilps xmm3, xmm3, -86
       vaddps   xmm3, xmm3, xmm4
       vmulss   xmm3, xmm3, xmm0
       vmovss   dword ptr [rcx], xmm3
       vmovaps  xmm3, xmm9
       vaddps   xmm2, xmm3, xmm2
       vmovaps  xmm3, xmm10
       vmovaps  xmm4, xmm2
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm6
       vmovshdup xmm5, xmm2
       vbroadcastss xmm5, xmm5
       vmulps   xmm4, xmm5, xmm4
       vaddps   xmm3, xmm4, xmm3
       vmovaps  xmm4, xmm8
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
 
G_M000_IG57:                ;; offset=0x29F2
       and      byte  ptr [rdx], al
       lea      rax, bword ptr [rbx+0x24]
       lea      rcx, bword ptr [rbx+0x48]
       lea      rdx, bword ptr [rbx+0x5F]
       vmovaps  xmm2, xmmword ptr [rbp-0x180]
       vmovaps  xmm3, xmm2
       vsubps   xmm3, xmm3, xmm14
       vinsertps xmm3, xmm3, xmm3, 56
       vinsertps xmm4, xmm13, xmm13, 56
       vmulps   xmm3, xmm4, xmm3
       vpermilps xmm4, xmm3, -11
       vaddps   xmm4, xmm4, xmm3
       vpermilps xmm3, xmm3, -86
       vaddps   xmm3, xmm3, xmm4
       vmulss   xmm0, xmm3, xmm0
       vmovss   dword ptr [rcx], xmm0
       vaddps   xmm0, xmm9, xmm2
       vmovaps  xmm2, xmm10
       vmovaps  xmm3, xmm0
       vbroadcastss xmm3, xmm3
       vmulps   xmm2, xmm3, xmm2
       vmovaps  xmm3, xmm6
       vmovshdup xmm4, xmm0
       vbroadcastss xmm4, xmm4
       vmulps   xmm3, xmm4, xmm3
       vaddps   xmm2, xmm3, xmm2
       vmovaps  xmm3, xmm8
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
       vmulps   xmm1, xmm1, xmm10
       vmovshdup xmm2, xmm0
       vbroadcastss xmm2, xmm2
       vmulps   xmm2, xmm2, xmm6
       vaddps   xmm1, xmm2, xmm1
       vunpckhps xmm0, xmm0, xmm0
       vbroadcastss xmm0, xmm0
       vmulps   xmm0, xmm0, xmm8
       vaddps   xmm0, xmm0, xmm1
       vmovsd   qword ptr [rax], xmm0
       vextractps dword ptr [rax+0x08], xmm0, 2
       xor      eax, eax
       mov      dword ptr [rbx+0x4C], eax
       mov      dword ptr [rbx+0x50], 1
       mov      dword ptr [rbx+0x54], 2
       mov      dword ptr [rbx+0x58], 3
 
G_M000_IG58:                ;; offset=0x2AFB
       vmovaps  xmm6, xmmword ptr [rsp+0x5C0]
       vmovaps  xmm7, xmmword ptr [rsp+0x5B0]
       vmovaps  xmm8, xmmword ptr [rsp+0x5A0]
       vmovaps  xmm9, xmmword ptr [rsp+0x590]
       vmovaps  xmm10, xmmword ptr [rsp+0x580]
       vmovaps  xmm11, xmmword ptr [rsp+0x570]
       vmovaps  xmm12, xmmword ptr [rsp+0x560]
       vmovaps  xmm13, xmmword ptr [rsp+0x550]
       vmovaps  xmm14, xmmword ptr [rsp+0x540]
       vmovaps  xmm15, xmmword ptr [rsp+0x530]
       add      rsp, 0x5D0
       pop      rbx
       pop      rsi
       pop      rdi
       pop      r14
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
RWD72  	dd	283424DCh		;     1e-14
RWD76  	dd	2B8CBCCCh		;     1e-12
RWD80  	dd	3F000000h		;       0.5
RWD84  	dd	469C395Dh		;   19996.7
RWD88  	dd	26901D7Dh		;     1e-15
RWD92  	dd	3CB851ECh		;    0.0225
RWD96  	dd	4234FED7h		;   45.2489

; Total bytes of code 11107
