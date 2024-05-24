#ifndef __STACK_H
#define __STACK_H
#include <stdio.h>
#define OK 1
#define ERROR 0
#define TRUE 1
#define FALSE 0
#define MAXSIZE 5
typedef double ElemType;
typedef int State;
typedef struct {  //˳��ջ�Ĵ洢�ṹ
    ElemType data[MAXSIZE];   //�����������ݣ����ΪMAXSIZE����Ϊջ������
    int top;   //����ջ��ָ��
} SqStack;

State initStack(SqStack *S);
int getLength(SqStack S);
State clearStack(SqStack *S);
State isEmpty(SqStack S);
State push(SqStack *S, ElemType e);
State pop(SqStack *S, ElemType *e);
void sum(SqStack *S, ElemType *e);
void avg(SqStack *S, ElemType *e);
void Stack_mid(SqStack *S, ElemType *e);
State getTop(SqStack S, ElemType *e);
State printStack(SqStack S);

#endif




























