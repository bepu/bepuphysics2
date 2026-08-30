; Assembly listing for method AosBaselines.ScalarDepthRefiner`4[BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar,BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar]:FindMinimumDepth(byref,byref,byref,byref,byref,float,float,byref,byref,byref,int) (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rbp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 67 single block inlinees; 8 inlinees without PGO data

G_M000_IG01:                ;; offset=0x0000
       push     rbp
       push     r15
       push     r14
       push     r13
       push     r12
       push     rdi
       push     rsi
       push     rbx
       sub      rsp, 408
       vmovaps  xmmword ptr [rsp+0x180], xmm6
       vmovaps  xmmword ptr [rsp+0x170], xmm7
       vmovaps  xmmword ptr [rsp+0x160], xmm8
       vmovaps  xmmword ptr [rsp+0x150], xmm9
       vmovaps  xmmword ptr [rsp+0x140], xmm10
       vmovaps  xmmword ptr [rsp+0x130], xmm11
       vmovaps  xmmword ptr [rsp+0x120], xmm12
       lea      rbp, [rsp+0x1D0]
       vxorps   xmm4, xmm4, xmm4
       vmovdqa  xmmword ptr [rbp-0x180], xmm4
       mov      rax, -192
       vmovdqa  xmmword ptr [rbp+rax-0xB0], xmm4
       vmovdqa  xmmword ptr [rbp+rax-0xA0], xmm4
       vmovdqa  xmmword ptr [rbp+rax-0x90], xmm4
       add      rax, 48
       jne      SHORT  -5 instr
       mov      rsi, rcx
       mov      rdi, rdx
       mov      r15, r8
       mov      rbx, r9
       mov      rdx, bword ptr [rbp+0x30]
       vmovss   xmm6, dword ptr [rbp+0x38]
       mov      r14, bword ptr [rbp+0x48]
       mov      r13, bword ptr [rbp+0x50]
       mov      r12d, dword ptr [rbp+0x60]
 
G_M000_IG02:                ;; offset=0x00B2
       inc      qword ptr [(reloc 0x7ff886b3b080)]
       vmovss   xmm0, dword ptr [rdx+0x04]
       vxorps   xmm1, xmm1, xmm1
       vucomiss xmm0, xmm1
       jbe      SHORT G_M000_IG04
 
G_M000_IG03:                ;; offset=0x00C8
       vmovss   xmm0, dword ptr [rsi+0x04]
       jmp      SHORT G_M000_IG05
 
G_M000_IG04:                ;; offset=0x00CF
       vmovss   xmm0, dword ptr [rsi+0x04]
       vxorps   xmm0, xmm0, xmmword ptr [reloc @RWD00]
 
G_M000_IG05:                ;; offset=0x00DC
       vxorps   xmm1, xmm1, xmm1
       vinsertps xmm0, xmm1, xmm0, 29
       vmovss   xmm1, dword ptr [rdx]
       vmulss   xmm1, xmm1, dword ptr [rdx]
       vmovss   xmm2, dword ptr [rdx+0x08]
       vmulss   xmm3, xmm2, xmm2
       vaddss   xmm1, xmm1, xmm3
       vsqrtss  xmm1, xmm1, xmm1
       vmovss   xmm3, dword ptr [rsi]
       vdivss   xmm3, xmm3, xmm1
       vmovss   xmm7, dword ptr [reloc @RWD16]
       vucomiss xmm1, xmm7
       seta     r8b
       movzx    r8, r8b
       test     r8d, r8d
       je       SHORT G_M000_IG07
 
G_M000_IG06:                ;; offset=0x0120
       vmulss   xmm1, xmm3, dword ptr [rdx]
       jmp      SHORT G_M000_IG08
 
G_M000_IG07:                ;; offset=0x0126
       vxorps   xmm1, xmm1, xmm1
 
G_M000_IG08:                ;; offset=0x012A
       vinsertps xmm0, xmm0, xmm1, 0
       test     r8d, r8d
       je       SHORT G_M000_IG10
 
G_M000_IG09:                ;; offset=0x0135
       vmulss   xmm1, xmm2, xmm3
       jmp      SHORT G_M000_IG11
 
G_M000_IG10:                ;; offset=0x013B
       vxorps   xmm1, xmm1, xmm1
 
G_M000_IG11:                ;; offset=0x013F
       vinsertps xmm0, xmm0, xmm1, 32
       vmovsd   xmm1, qword ptr [rdx]
       vinsertps xmm1, xmm1, dword ptr [rdx+0x08], 40
       vbroadcastss xmm8, dword ptr [reloc @RWD00]
       vxorps   xmm2, xmm8, xmm1
       vmovsd   xmm3, qword ptr [rbx]
       vinsertps xmm3, xmm3, dword ptr [rbx+0x08], 40
       vmovsd   xmm4, qword ptr [rbx+0x0C]
       vinsertps xmm4, xmm4, dword ptr [rbx+0x14], 40
       vmovsd   xmm5, qword ptr [rbx+0x18]
       vinsertps xmm5, xmm5, dword ptr [rbx+0x20], 40
       vinsertps xmm2, xmm2, xmm2, 56
       vinsertps xmm9, xmm3, xmm3, 56
       vmulps   xmm9, xmm9, xmm2
       vpermilps xmm10, xmm9, -11
       vaddps   xmm10, xmm10, xmm9
       vpermilps xmm9, xmm9, -86
       vaddps   xmm9, xmm9, xmm10
       vinsertps xmm10, xmm4, xmm4, 56
       vmulps   xmm10, xmm10, xmm2
       vpermilps xmm11, xmm10, -11
       vaddps   xmm11, xmm11, xmm10
       vpermilps xmm10, xmm10, -86
       vaddps   xmm10, xmm10, xmm11
       vinsertps xmm11, xmm5, xmm5, 56
       vmulps   xmm2, xmm11, xmm2
       vinsertps xmm9, xmm9, xmm10, 16
       vpermilps xmm10, xmm2, -11
       vaddps   xmm10, xmm10, xmm2
       vpermilps xmm2, xmm2, -86
       vaddps   xmm2, xmm2, xmm10
       vinsertps xmm2, xmm9, xmm2, 40
       vmovshdup xmm9, xmm2
       vxorps   xmm10, xmm10, xmm10
       vucomiss xmm9, xmm10
       jbe      SHORT G_M000_IG13
 
G_M000_IG12:                ;; offset=0x0201
       vmovss   xmm9, dword ptr [rdi+0x04]
       jmp      SHORT G_M000_IG14
 
G_M000_IG13:                ;; offset=0x0208
       vmovss   xmm9, dword ptr [rdi+0x04]
       vxorps   xmm9, xmm9, xmmword ptr [reloc @RWD00]
 
G_M000_IG14:                ;; offset=0x0215
       vxorps   xmm10, xmm10, xmm10
       vinsertps xmm9, xmm10, xmm9, 29
       vmovaps  xmm10, xmm2
       vmovaps  xmm11, xmm2
       vmulss   xmm10, xmm10, xmm11
       vunpckhps xmm11, xmm2, xmm2
       vunpckhps xmm12, xmm2, xmm2
       vmulss   xmm11, xmm11, xmm12
       vaddss   xmm10, xmm10, xmm11
       vsqrtss  xmm10, xmm10, xmm10
       vmovss   xmm11, dword ptr [rdi]
       vdivss   xmm11, xmm11, xmm10
       vucomiss xmm10, xmm7
       seta     r8b
       movzx    r8, r8b
       test     r8d, r8d
       je       SHORT G_M000_IG16
 
G_M000_IG15:                ;; offset=0x025E
       vmovaps  xmm10, xmm2
       vmulss   xmm10, xmm10, xmm11
       jmp      SHORT G_M000_IG17
 
G_M000_IG16:                ;; offset=0x0269
       vxorps   xmm10, xmm10, xmm10
 
G_M000_IG17:                ;; offset=0x026E
       vinsertps xmm9, xmm9, xmm10, 0
       test     r8d, r8d
       je       SHORT G_M000_IG19
 
G_M000_IG18:                ;; offset=0x0279
       vunpckhps xmm2, xmm2, xmm2
       vmulss   xmm2, xmm2, xmm11
       jmp      SHORT G_M000_IG20
 
G_M000_IG19:                ;; offset=0x0284
       vxorps   xmm2, xmm2, xmm2
 
G_M000_IG20:                ;; offset=0x0288
       vinsertps xmm2, xmm9, xmm2, 32
       vmovaps  xmm9, xmm2
       vbroadcastss xmm9, xmm9
       vmovshdup xmm10, xmm2
       vbroadcastss xmm10, xmm10
       vunpckhps xmm2, xmm2, xmm2
       vbroadcastss xmm2, xmm2
       vmulps   xmm3, xmm9, xmm3
       vmulps   xmm4, xmm10, xmm4
       vaddps   xmm3, xmm4, xmm3
       vmulps   xmm2, xmm2, xmm5
       vaddps   xmm2, xmm2, xmm3
       vmovsd   xmm3, qword ptr [r15]
       vinsertps xmm3, xmm3, dword ptr [r15+0x08], 40
       vaddps   xmm2, xmm3, xmm2
       vmovaps  xmm3, xmm0
       vsubps   xmm2, xmm3, xmm2
       vmovaps  xmm3, xmm2
       vinsertps xmm3, xmm3, xmm3, 56
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm1, xmm1, xmm3
       vpermilps xmm3, xmm1, -11
       vaddps   xmm3, xmm3, xmm1
       vpermilps xmm1, xmm1, -86
       vaddps   xmm1, xmm1, xmm3
       vmovsd   qword ptr [rbp-0x118], xmm2
       vextractps dword ptr [rbp-0x110], xmm2, 2
       vmovsd   qword ptr [rbp-0xF8], xmm2
       vextractps dword ptr [rbp-0xF0], xmm2, 2
       vmovsd   qword ptr [rbp-0xD8], xmm2
       vextractps dword ptr [rbp-0xD0], xmm2, 2
       vmovsd   qword ptr [rbp-0x10C], xmm0
       vextractps dword ptr [rbp-0x104], xmm0, 2
       vmovsd   qword ptr [rbp-0xEC], xmm0
       vextractps dword ptr [rbp-0xE4], xmm0, 2
       vmovsd   qword ptr [rbp-0xCC], xmm0
       vextractps dword ptr [rbp-0xC4], xmm0, 2
       mov      byte  ptr [rbp-0xFC], 1
       mov      byte  ptr [rbp-0xDC], 0
       mov      byte  ptr [rbp-0xBC], 0
       vmovss   xmm9, dword ptr [rbp+0x40]
       vmovsd   xmm0, qword ptr [rdx]
       vinsertps xmm0, xmm0, dword ptr [rdx+0x08], 40
       vmovsd   qword ptr [r13], xmm0
       vextractps dword ptr [r13+0x08], xmm0, 2
       vmovss   dword ptr [r14], xmm1
       vucomiss xmm9, xmm1
       ja       G_M000_IG44
 
G_M000_IG21:                ;; offset=0x03AA
       xor      edx, edx
       mov      dword ptr [rbp-0x120], edx
       vxorps   xmm0, xmm0, xmm0
       vmovaps  xmmword ptr [rbp-0x140], xmm0
       vmovaps  xmmword ptr [rbp-0x150], xmm0
       lea      rdx, [rbp-0x120]
       mov      qword ptr [rsp+0x20], rdx
       mov      bword ptr [rsp+0x28], r13
       vmovss   xmm0, dword ptr [r14]
       vmovss   dword ptr [rsp+0x30], xmm0
       vmovss   dword ptr [rsp+0x38], xmm6
       lea      rdx, [rbp-0x130]
       mov      qword ptr [rsp+0x40], rdx
       lea      rdx, [rbp-0x140]
       lea      r8, [rbp-0x150]
       lea      rcx, [rbp-0x118]
       xor      r9d, r9d
       call     [AosBaselines.ScalarDepthRefiner`4[BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar,BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar]:GetNextNormal(byref,byref,byref,bool,byref,byref,float,float,byref)]
       xor      eax, eax
       mov      dword ptr [rbp-0x154], eax
       cmp      eax, r12d
       jge      G_M000_IG43
 
G_M000_IG22:                ;; offset=0x0423
       cmp      byte  ptr [rbp-0x120], 0
       jne      G_M000_IG43
       mov      rdx, 0x7FF886B3B088
       inc      qword ptr [rdx]
       vmovss   xmm0, dword ptr [rbp-0x12C]
       vxorps   xmm1, xmm1, xmm1
       vucomiss xmm0, xmm1
       jbe      SHORT G_M000_IG24
 
G_M000_IG23:                ;; offset=0x044F
       vmovss   xmm0, dword ptr [rsi+0x04]
       jmp      SHORT G_M000_IG25
 
G_M000_IG24:                ;; offset=0x0456
       vmovss   xmm0, dword ptr [rsi+0x04]
       vxorps   xmm0, xmm0, xmmword ptr [reloc @RWD00]
 
G_M000_IG25:                ;; offset=0x0463
       vmovaps  xmm1, xmmword ptr [rbp-0x180]
       vinsertps xmm0, xmm1, xmm0, 16
       vmovaps  xmmword ptr [rbp-0x180], xmm0
       vmovss   xmm0, dword ptr [rbp-0x130]
       vmulss   xmm1, xmm0, xmm0
       vmovss   xmm2, dword ptr [rbp-0x128]
       vmulss   xmm2, xmm2, xmm2
       vaddss   xmm1, xmm1, xmm2
       vsqrtss  xmm1, xmm1, xmm1
       vmovss   xmm2, dword ptr [rsi]
       vdivss   xmm2, xmm2, xmm1
       vucomiss xmm1, xmm7
       seta     dl
       movzx    rdx, dl
       test     edx, edx
       je       SHORT G_M000_IG27
 
G_M000_IG26:                ;; offset=0x04AF
       vmulss   xmm0, xmm0, xmm2
       jmp      SHORT G_M000_IG28
 
G_M000_IG27:                ;; offset=0x04B5
       vxorps   xmm0, xmm0, xmm0
 
G_M000_IG28:                ;; offset=0x04B9
       vmovaps  xmm1, xmmword ptr [rbp-0x180]
       vinsertps xmm0, xmm1, xmm0, 0
       vmovaps  xmmword ptr [rbp-0x180], xmm0
       test     edx, edx
       je       SHORT G_M000_IG30
 
G_M000_IG29:                ;; offset=0x04D3
       vmulss   xmm0, xmm2, dword ptr [rbp-0x128]
       jmp      SHORT G_M000_IG31
 
G_M000_IG30:                ;; offset=0x04DD
       vxorps   xmm0, xmm0, xmm0
 
G_M000_IG31:                ;; offset=0x04E1
       vmovaps  xmm1, xmmword ptr [rbp-0x180]
       vinsertps xmm0, xmm1, xmm0, 32
       vmovaps  xmmword ptr [rbp-0x180], xmm0
       vmovaps  xmm0, xmmword ptr [rbp-0x130]
       vxorps   xmm0, xmm0, xmm8
       vmovsd   xmm1, qword ptr [rbx]
       vinsertps xmm1, xmm1, dword ptr [rbx+0x08], 40
       vmovsd   xmm2, qword ptr [rbx+0x0C]
       vinsertps xmm2, xmm2, dword ptr [rbx+0x14], 40
       vmovsd   xmm3, qword ptr [rbx+0x18]
       vinsertps xmm3, xmm3, dword ptr [rbx+0x20], 40
       vinsertps xmm0, xmm0, xmm0, 56
       vinsertps xmm4, xmm1, xmm1, 56
       vmulps   xmm4, xmm4, xmm0
       vpermilps xmm5, xmm4, -11
       vaddps   xmm5, xmm5, xmm4
       vpermilps xmm4, xmm4, -86
       vaddps   xmm4, xmm4, xmm5
       vinsertps xmm5, xmm2, xmm2, 56
       vmulps   xmm5, xmm5, xmm0
       vpermilps xmm10, xmm5, -11
       vaddps   xmm10, xmm10, xmm5
       vpermilps xmm5, xmm5, -86
       vaddps   xmm5, xmm5, xmm10
       vinsertps xmm10, xmm3, xmm3, 56
       vmulps   xmm0, xmm10, xmm0
       vinsertps xmm4, xmm4, xmm5, 16
       vpermilps xmm5, xmm0, -11
       vaddps   xmm5, xmm5, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm5
       vinsertps xmm0, xmm4, xmm0, 40
       vmovshdup xmm4, xmm0
       vxorps   xmm5, xmm5, xmm5
       vucomiss xmm4, xmm5
       jbe      SHORT G_M000_IG33
 
G_M000_IG32:                ;; offset=0x05A2
       vmovss   xmm4, dword ptr [rdi+0x04]
       jmp      SHORT G_M000_IG34
 
G_M000_IG33:                ;; offset=0x05A9
       vmovss   xmm4, dword ptr [rdi+0x04]
       vxorps   xmm4, xmm4, xmmword ptr [reloc @RWD00]
 
G_M000_IG34:                ;; offset=0x05B6
       vxorps   xmm5, xmm5, xmm5
       vinsertps xmm4, xmm5, xmm4, 29
       vmovaps  xmm5, xmm0
       vmulss   xmm10, xmm5, xmm5
       vunpckhps xmm0, xmm0, xmm0
       vmulss   xmm11, xmm0, xmm0
       vaddss   xmm10, xmm10, xmm11
       vsqrtss  xmm10, xmm10, xmm10
       vmovss   xmm11, dword ptr [rdi]
       vdivss   xmm11, xmm11, xmm10
       vucomiss xmm10, xmm7
       seta     dl
       movzx    rdx, dl
       test     edx, edx
       je       SHORT G_M000_IG36
 
G_M000_IG35:                ;; offset=0x05F1
       vmulss   xmm5, xmm5, xmm11
       jmp      SHORT G_M000_IG37
 
G_M000_IG36:                ;; offset=0x05F8
       vxorps   xmm5, xmm5, xmm5
 
G_M000_IG37:                ;; offset=0x05FC
       vinsertps xmm4, xmm4, xmm5, 0
       test     edx, edx
       je       SHORT G_M000_IG39
 
G_M000_IG38:                ;; offset=0x0606
       vmulss   xmm0, xmm0, xmm11
       jmp      SHORT G_M000_IG40
 
G_M000_IG39:                ;; offset=0x060D
       vxorps   xmm0, xmm0, xmm0
 
G_M000_IG40:                ;; offset=0x0611
       vinsertps xmm0, xmm4, xmm0, 32
       vmovaps  xmm4, xmm0
       vbroadcastss xmm4, xmm4
       vmovshdup xmm5, xmm0
       vbroadcastss xmm5, xmm5
       vunpckhps xmm0, xmm0, xmm0
       vbroadcastss xmm0, xmm0
       vmulps   xmm1, xmm4, xmm1
       vmulps   xmm2, xmm5, xmm2
       vaddps   xmm1, xmm2, xmm1
       vmulps   xmm0, xmm0, xmm3
       vaddps   xmm0, xmm0, xmm1
       vmovsd   xmm1, qword ptr [r15]
       vinsertps xmm1, xmm1, dword ptr [r15+0x08], 40
       vaddps   xmm0, xmm1, xmm0
       vmovaps  xmm1, xmmword ptr [rbp-0x180]
       vsubps   xmm0, xmm1, xmm0
       vmovaps  xmmword ptr [rbp-0x170], xmm0
       vmovaps  xmm0, xmmword ptr [rbp-0x170]
       vinsertps xmm0, xmm0, xmm0, 56
       vmovaps  xmm1, xmmword ptr [rbp-0x130]
       vinsertps xmm1, xmm1, xmm1, 56
       vmulps   xmm0, xmm1, xmm0
       vpermilps xmm1, xmm0, -11
       vaddps   xmm1, xmm1, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm1
       vmovss   xmm1, dword ptr [r14]
       vucomiss xmm1, xmm0
       jbe      SHORT G_M000_IG42
 
G_M000_IG41:                ;; offset=0x06A9
       vmovss   dword ptr [r14], xmm0
       vmovaps  xmm0, xmmword ptr [rbp-0x130]
       vmovsd   qword ptr [r13], xmm0
       vextractps dword ptr [r13+0x08], xmm0, 2
 
G_M000_IG42:                ;; offset=0x06C3
       vmovss   xmm0, dword ptr [r14]
       vucomiss xmm9, xmm0
       jae      SHORT G_M000_IG43
       lea      rdx, [rbp-0x120]
       mov      qword ptr [rsp+0x20], rdx
       mov      bword ptr [rsp+0x28], r13
       vmovss   dword ptr [rsp+0x30], xmm0
       vmovss   dword ptr [rsp+0x38], xmm6
       lea      rdx, [rbp-0x130]
       mov      qword ptr [rsp+0x40], rdx
       lea      rdx, [rbp-0x170]
       lea      r8, [rbp-0x180]
       lea      rcx, [rbp-0x118]
       mov      r9d, 1
       call     [AosBaselines.ScalarDepthRefiner`4[BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar,BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar]:GetNextNormal(byref,byref,byref,bool,byref,byref,float,float,byref)]
       mov      eax, dword ptr [rbp-0x154]
       inc      eax
       cmp      eax, r12d
       mov      dword ptr [rbp-0x154], eax
       jl       G_M000_IG22
 
G_M000_IG43:                ;; offset=0x072F
       vmovss   xmm0, dword ptr [reloc @RWD20]
       vdivss   xmm0, xmm0, dword ptr [rbp-0xB8]
       vmovsd   xmm1, qword ptr [rbp-0x10C]
       vinsertps xmm1, xmm1, dword ptr [rbp-0x104], 40
       vmulss   xmm2, xmm0, dword ptr [rbp-0x100]
       vbroadcastss xmm2, xmm2
       vmulps   xmm1, xmm2, xmm1
       vmovsd   xmm2, qword ptr [rbp-0xEC]
       vinsertps xmm2, xmm2, dword ptr [rbp-0xE4], 40
       vmulss   xmm3, xmm0, dword ptr [rbp-0xE0]
       vbroadcastss xmm3, xmm3
       vmulps   xmm2, xmm3, xmm2
       vmovsd   xmm3, qword ptr [rbp-0xCC]
       vinsertps xmm3, xmm3, dword ptr [rbp-0xC4], 40
       vmulss   xmm0, xmm0, dword ptr [rbp-0xC0]
       vbroadcastss xmm0, xmm0
       vmulps   xmm0, xmm0, xmm3
       vaddps   xmm1, xmm2, xmm1
       mov      rbx, bword ptr [rbp+0x58]
       vmovsd   qword ptr [rbx], xmm1
       vextractps dword ptr [rbx+0x08], xmm1, 2
       vmovsd   xmm1, qword ptr [rbx]
       vinsertps xmm1, xmm1, dword ptr [rbx+0x08], 40
       vaddps   xmm0, xmm1, xmm0
       vmovsd   qword ptr [rbx], xmm0
       vextractps dword ptr [rbx+0x08], xmm0, 2
       jmp      SHORT G_M000_IG45
 
G_M000_IG44:                ;; offset=0x07D7
       vxorps   xmm0, xmm0, xmm0
       mov      rax, bword ptr [rbp+0x58]
       vmovsd   qword ptr [rax], xmm0
       vmovss   dword ptr [rax+0x08], xmm0
 
G_M000_IG45:                ;; offset=0x07E8
       vmovaps  xmm6, xmmword ptr [rsp+0x180]
       vmovaps  xmm7, xmmword ptr [rsp+0x170]
       vmovaps  xmm8, xmmword ptr [rsp+0x160]
       vmovaps  xmm9, xmmword ptr [rsp+0x150]
       vmovaps  xmm10, xmmword ptr [rsp+0x140]
       vmovaps  xmm11, xmmword ptr [rsp+0x130]
       vmovaps  xmm12, xmmword ptr [rsp+0x120]
       add      rsp, 408
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
RWD20  	dd	3F800000h		;         1

; Total bytes of code 2107
