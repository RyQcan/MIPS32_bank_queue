#$t1   新分配的编号,
#$t2   当前总人数,
#$t3   当前服务人的编号 
#5632  队伍首地址
#$t7   队长标记位，$t1≤99时，$t7=0，$t1＞99时，$t7=1
#$s5   防抖标志
 
.text
	.globl	main
main:
    addi $t1, $zero, 0     #新分配的编号
    addi $t2, $zero, 0     #总人数 
    addi $t3, $zero, 1     #下一个显示的人
    addi $s3, $zero, 100
    addi $t7, $zero, 0     #100 flag
    addi $s5, $zero, 0  #flag   
   
    j .A_SHOW  

DOWN:
    #插队
    bne $s5,$zero,.D_N_LEFT 
    addi $s5, $zero,1
    lw	$t6,0x400c($zero) #switch
    lb  $t8,5632($t6)
    beq $t8,$zero,.D_SHOW
    sb  $zero,5632($t6)     #插队人的对应的值为0
    addi $t2, $t2, -1
     
.D_SHOW:
     #前两个灯显示当前服务的编号t6
    andi $a0, $t6, 240      #t6和11110000按位与运算
    srl  $a0, $a0, 4        #逻辑右移4位
    addi $a1, $zero,7       #0b0111
    sb   $a1, 0x4014($zero)  #选灯
    jal MATCH        
	jal WAIT
  
    andi $a0, $t6, 15       #t6和00001111按位与运算
    addi $a1, $zero,11      #0b1011
    sb   $a1, 0x4014($zero)  #选灯
    jal MATCH  
    jal WAIT

    #后两个灯显示剩余队长t2  
    andi $a0, $t2, 240      #t2和11110000按位与运算
    srl  $a0, $a0, 4        #逻辑右移4位
    addi $a1, $zero,13       #0b1101
    sb   $a1, 0x4014($zero)  #选灯
    jal MATCH           
    jal WAIT

    andi $a0, $t2, 15       #t2和00001111按位与运算
    addi $a1, $zero,14      #0b1110
    sb   $a1, 0x4014($zero)  #选灯
    jal MATCH  
    jal WAIT
    
.D_N_LEFT:
    lw	$s0,0x4010($zero)#left
	beq	$s0,$zero,.D_N_UP
	nop
    j LEFT

.D_N_UP:
    lw	$s0,0x4000($zero) #up
	beq	$s0,$zero,.D_N_DOWN
	nop
    j UPUP

.D_N_DOWN:
    lw	$s0,0x4008($zero) #down
	beq	$s0,$zero,.D_N_NONE 
	nop
    j DOWN

.D_N_NONE:
	addi   $s5,$zero,0
    j      .L7
	

UPUP:
   #下一位顾客得到服务
    bne $s5,$zero,.B_N_LEFT 
    addi $s5, $zero,1
    beq $t1, $zero, AAA
    j BBB
    AAA:
    add $t6,$zero,$zero
    BBB:
    beq $t2,$zero,.B_SHOW 
    addi $t2, $t2, -1 #人数--
    lb $t4, 5632($t3)
    beq $t4,$zero, .NEXT4
    j .NEXT5
.NEXT4:
    beq  $t3, $t1, .B_SHOW
    addi $t3,$t3,1
.NEXT5:
    sb   $zero,5632($t3) #t3出队,t3地址的值变为0
    add $t6,$zero,$t3

    #重新定位t3的位置
    beq  $t3, $t1, .B_SHOW
    addi $t3, $t3, 1   #下一位
    bne $t3, $s3, .NEXT3
    addi $t3,$zero, 1 #等于100时 置为1
.NEXT3:
    sub  $v0, $t1, $t3 #v0中存(t1-t3)
    bltz $v0, .NEXT6   #<0则跳转
    beq $v0,$zero, .B_SHOW
    j NEXT1
.NEXT6:
    addi $v0,$v0,99  #若t1<t3,循环t1+99-t3次
    NEXT1: 
        lbu $v1,5632($t3)    #取出t3对应内存的值
        bne $v1,$zero,ELSE   
        addi $t3,$t3,1
        beq $t3,$s3,T3_1    #判断t3是否加到100,加到100则置为1
        j NEXT2
        T3_1:
           addi $t3,$t3,-99
        j NEXT2

        ELSE:
            j .B_SHOW

        NEXT2:
            addi $v0,$v0,-1 
            bne $v0,$zero,NEXT1
    j .B_SHOW


.B_SHOW:
     #前两个灯显示当前服务的编号t6
    andi $a0, $t6, 240      #t6和11110000按位与运算
    srl  $a0, $a0, 4        #逻辑右移4位
    addi $a1, $zero,7       #0b0111
    sb   $a1, 0x4014($zero)  #选灯
    jal MATCH        
	jal WAIT
 
    andi $a0, $t6, 15       #t6和00001111按位与运算
    addi $a1, $zero,11      #0b1011
    sb   $a1, 0x4014($zero)  #选灯
    jal MATCH  
    jal WAIT

    #后两个灯显示剩余队长t2  
    andi $a0, $t2, 240      #t2和11110000按位与运算
    srl  $a0, $a0, 4        #逻辑右移4位
    addi $a1, $zero,13       #0b1101
    sb   $a1, 0x4014($zero)  #选灯
    jal MATCH           
    jal WAIT

    andi $a0, $t2, 15       #t2和00001111按位与运算
    addi $a1, $zero,14      #0b1110
    sb   $a1, 0x4014($zero)  #选灯
    jal MATCH  
    jal WAIT
    
.B_N_LEFT:
    lw	$s0,0x4010($zero)#left
	beq	$s0,$zero,.B_N_UP
	nop
    j LEFT

.B_N_UP:
    lw	$s0,0x4000($zero) #up
	beq	$s0,$zero,.B_N_DOWN 
	nop
    j UPUP

.B_N_DOWN:
    lw	$s0,0x4008($zero) #down
	beq	$s0,$zero,.B_N_NONE 
	nop
    j DOWN

.B_N_NONE:
	addi   $s5,$zero,0
    j      .L4

LEFT:
    bne  $s5,$zero,.A_N_LEFT  #s5=0,表示可以接受按键
    addi $s5, $zero, 1         #按键被按下，s5=1，此时不可以继续接受按键，达到防抖功能
    addi $t1, $t1, 1  
    
    addi $a0, $zero, 100               #max people
    beq  $t1,$a0,FLASH    

CHECK:
    beq  $t7,$zero,GO
    lbu  $t4,5632($t1)      
    bne  $t4,$zero,FULL 
    bne  $t1,$t3,GO #t1
    #重新定位t3  
    addi $t3, $t3, 1   #下一位
    bne  $t3, $s3, .NEXT03
    addi $t3,$zero, 1 #等于100时 置为1
.NEXT03:
    addi $v0, $zero, 98
    NEXT01: 
        lbu $v1,5632($t3)    #取出t3对应内存的值
        bne $v1,$zero,ELSE1   
        addi $t3,$t3,1
        beq $t3,$s3,T3_01    #判断t3是否加到100,加到100则置为1
        j NEXT02
        T3_01:
           addi $t3,$t3,-99
        j NEXT02

        ELSE1:
            j GO

        NEXT02:
            addi $v0,$v0,-1 
            bne $v0,$zero,NEXT01
    
GO:
    addi $t2, $t2, 1         
    sb	 $t1, 5632($t1)	   
  

#左键的显示以及检测按键
.A_SHOW: 
    andi $a0, $t1, 240     
    srl  $a0, $a0, 4       
    addi $a1, $zero,7   #0111   
    sb   $a1, 0x4014($zero)  
    jal MATCH                  
    jal WAIT

    andi $a0, $t1, 15       
    addi $a1, $zero,11  #1011   
    sb   $a1, 0x4014($zero) 
    jal MATCH                 
    jal WAIT

    andi $a0, $t2, 240     
    srl  $a0, $a0, 4       
    addi $a1, $zero,13   #1101    
    sb   $a1, 0x4014($zero)
    jal MATCH   
    jal WAIT
        
    #show t2 second
    andi $a0, $t2, 15       
    addi $a1, $zero,14     #1110 
    sb   $a1, 0x4014($zero) 
    jal MATCH 
    jal WAIT

.A_N_LEFT:
    lw	$s0,0x4010($zero)     #left
	beq	$s0,$zero,.A_N_UP
	nop
    j LEFT

.A_N_UP:
    lw	$s0,0x4000($zero)     #up
	beq	$s0,$zero,.A_N_DOWN
	nop
    j UPUP

.A_N_DOWN:
    lw	$s0,0x4008($zero) #down
	beq	$s0,$zero,.A_N_NONE 
	nop
    j DOWN

.A_N_NONE:
	addi  $s5,$zero,0  #s5=0,表示可以接受按键
	j     .L1


FLASH:
    addi $t7,$zero,1    #t7判断队伍有没有满     
    addi $t1,$t1,-99       
    j CHECK

FULL:
    addi $t6, $zero, 1
    beq  $t1,$t6, EQ
    addi $t1,$t1, -1
    j    .A_SHOW
EQ:
    addi $t1,$zero, 99
    j    .A_SHOW 


MATCH:
    addi $sp,$sp,-12      
    sw   $a0,4($sp)      
    sw   $a1,0($sp)       

	addi $a1,$zero,0
    beq $a0,$a1,C0
    addi $a1,$zero,1
    beq $a0,$a1,C1	
    addi $a1,$zero,2
    beq $a0,$a1,C2	
    addi $a1,$zero,3
    beq $a0,$a1,C3	
    addi $a1,$zero,4
    beq $a0,$a1,C4	
    addi $a1,$zero,5
    beq $a0,$a1,C5	
    addi $a1,$zero,6
    beq $a0,$a1,C6	
    addi $a1,$zero,7
    beq $a0,$a1,C7	
    addi $a1,$zero,8
    beq $a0,$a1,C8
    addi $a1,$zero,9
    beq $a0,$a1,C9	
    addi $a1,$zero,10
    beq $a0,$a1,CA
    addi $a1,$zero,11
    beq $a0,$a1,CB	
    addi $a1,$zero,12
    beq $a0,$a1,CC	
    addi $a1,$zero,13
    beq $a0,$a1,CD	
    addi $a1,$zero,14
    beq $a0,$a1,CE	
    addi $a1,$zero,15
    beq $a0,$a1,CF
    
C0:
    addi $s4,$zero,1   
    sb   $s4,0x4018($zero)
    j    recover
C1:
    addi $s4,$zero,79  
    sb   $s4,0x4018($zero)
    j    recover
C2:
    addi $s4,$zero,18 
    sb   $s4,0x4018($zero)
    j    recover
C3:
    addi $s4,$zero,6   
    sb   $s4,0x4018($zero)
    j    recover
C4:
    addi $s4,$zero,76  
    sb   $s4,0x4018($zero)
    j    recover
C5:
    addi $s4,$zero,36  
    sb   $s4,0x4018($zero)
    j    recover
C6:
    addi $s4,$zero,32  
    sb   $s4,0x4018($zero)
    j    recover
C7:
    addi $s4,$zero,15  
    sb   $s4,0x4018($zero)
    j    recover
C8:
    addi $s4,$zero,0  
    sb   $s4,0x4018($zero)
    j    recover
C9:
    addi $s4,$zero,4   
    sb   $s4,0x4018($zero)
    j    recover
CA:
    addi $s4,$zero,8   
    sb   $s4,0x4018($zero)
    j    recover
CB:
    addi $s4,$zero,96   
    sb   $s4,0x4018($zero)
    j    recover
CC:
    addi $s4,$zero,49  
    sb   $s4,0x4018($zero)
    j    recover
CD:
    addi $s4,$zero,66   
    sb   $s4,0x4018($zero)
    j    recover
CE:
    addi $s4,$zero,48  
    sb   $s4,0x4018($zero)
    j    recover
CF:
    addi $s4,$zero,56  
    sb   $s4,0x4018($zero)
    j    recover


recover:
    lw $a1,0($sp)        
    lw $a0,4($sp)
    addi $sp,$sp,12     
    jr $ra
	

.L1: 
	addi $s0, $zero, 15 
	sw	$s0,0x4014($zero)  
	addi, $s0, $zero, 0xff
	sw	$s0,0x4018($zero)


	addi $s0, $zero, 0
	addi $s1, $zero, 10000
	
.L2:	
	beq	$s0,$s1,.A_SHOW 
	nop
	
	addi	$s0,$s0,1 
	j	.L2
	nop

.L4: 
	addi $s0, $zero, 15 
	sw	$s0,0x4014($zero)  
	addi $s0, $zero, 0xff
	sw	$s0,0x4018($zero)


	addi $s0, $zero, 0
	addi $s1, $zero, 10000
	
.L5:	
	beq	$s0,$s1,.B_SHOW 
	nop
	
	addi	$s0,$s0,1 
	j	.L5
	nop

.L7: #什么都不显示，更新t0=0，t1=10000
	addi $s0, $zero, 15
	sw	$s0,0x4014($zero)  
	addi $s0, $zero, 0xff
	sw	$s0,0x4018($zero)


	addi $s0, $zero, 0
	addi $s1, $zero, 10000
	
.L8:	#分频
	beq	$s0,$s1,.D_SHOW 
	nop
	
	addi	$s0,$s0,1 
	j	.L8
	nop


#子程序
WAIT:
    addi $sp,$sp,-12
    sw   $s0,4($sp)
    sw   $s1,0($sp)

    addi $s0, $zero, 0     #和L4一起 放空1000拍
    addi $s1, $zero, 1000
        
    .W0:	
        beq	$s0,$s1,.W1  #放空完1000拍跳转.L5
        nop
        
        addi	$s0,$s0,1
        j	.W0
        nop
        
    .W1:	
        addi $s0, $zero, 0xff   
        sw	$s0,0x4018($zero) #把清零

    lw   $s1,0($sp)
    lw   $s0,4($sp)
    addi $sp,$sp,12
    jr   $ra