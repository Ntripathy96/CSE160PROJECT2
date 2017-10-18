/*
 * Author: UCM ANDES Lab
 * Description: A simple method of converting the hashmap class I created into an iterator.
 */
 #ifndef ITERATOR_H
 #define ITERATOR_H
 
 #include "hashmap.h"
 
 typedef struct iterator{
     hashType values[HASH_MAX_SIZE];
     uint16_t size;
     uint16_t position;
 }iterator;
 
 /*
  * iteratorInit - copies values from the hashmap to the iterator struct.
  * @param
  * 			iterator *it = iterator that is to be made.
  * 			hashmap *input = hashmap from which the values are made from.
  */
 void iteratorInit(iterator *it, hashmap *input){
     uint16_t i;
     it->position = 0;
     it->size = 0;
         
     for(i=0; i<input->numofVals; i++){
         it->values[i] = hashmapGet(input, input->keys[i]);
         it->size++;
     }
 }
 
 /*
  * iteratorNext - returns the next value.
  * @param
  * 			iterator *it = iterator that the value is getting from.
  */
 hashType iteratorNext(iterator *it){
     if(it->position < it->size){
         hashType temp=it->values[it->position];
         it->position++;
         return temp;
     }
     it->position++;
     dbg("iterator", "Error: iterator has overflown.");
     return it->values[0];
 }
 
 void iteratorResetPos(iterator *it){
     it->position=0;	
 }
 
 /*
  * routeIteratorNext - returns a bool if the iterator has a next value.
  * @param
  * 			iterator *it = iterator that the value is getting from.
  * @return
  * 			bool = returns TRUE if there is another value else returns FALSE.
  */
 bool iteratorHasNext(iterator *it){
     dbg("iterator", "it Position: %hhu \nit Size: %hhu\n", it->position, it->size);
     if(it->position < it->size) return TRUE;
     return FALSE;
 }
 #endif /* ITERATOR_H */