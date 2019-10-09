#include "shared_memory.h"


#define SLEEP_TEST 2
#define SLEEP_TEST_UM 100000


// Opens shared memory for Consumer
template <typename T>
int SharedMemory<T>::openSharedMemory(){
	if(_type!=SharedMemoryType::Consumer){
		return ERROR;
	}
	_shm=shared_memory_object(open_only, _sharedMemoryName, read_write);

	//Map the whole shared memory in this process
	_region= mapped_region(_shm,read_write);

	//get the region address
	_addr = _region.get_address();

	//Obtain the shared structure
	_sharedMemory = static_cast<shared_memory_buffer_float<T>*>(_addr);
//  _sharedMemory = new (_addr) shared_memory_buffer_float;

	return SUCCESS;
}


// Opens shared memory for Producer - creates the share memory, if it already exists opens
template <typename T>
int SharedMemory<T>::openOrCreateSharedMemory(){
	// Only producer can create new shared memory
	if(_type!=SharedMemoryType::Producer){
		return ERROR;
	}

	//use old shared memory if exists else create a new one
	_shm=shared_memory_object(open_or_create, _sharedMemoryName, read_write);

	//set the size of the memory object
	_shm.truncate(sizeof(shared_memory_buffer_float<T>));

	//map the whole shared memory in this process
	_region= mapped_region(_shm,read_write);

	//get the region address
	_addr = _region.get_address();

	//create a shared memory buffer in memory
	_sharedMemory = new (_addr) shared_memory_buffer_float<T>;

	return SUCCESS;
}

// ###########################################
// Reads shared memory
template<typename T>
T SharedMemory<T>::readSynchronously(){

	T tmp;

	// Only consumer may read from the shared memory
	if(_type!=SharedMemoryType::Consumer){
		assert(0);
	}

	// waits until the producer signals that the consumer may read
	_sharedMemory->reader.wait();

	// here the process reads data from the shared memory!
	tmp=_sharedMemory->msg;

  	//writer can write
	_sharedMemory->writer.post(); // signals that the writer can write

	return tmp;

}

// ###########################################
template <typename T>
void SharedMemory<T>::writeSynchronously(T msg_tmp){
	// Only producer can write to the shared memory
	if(_type!=SharedMemoryType::Producer){
		assert(0);
	}

	if(_sharedMemory==NULL){
		assert(0);
	}

	// wait for signal that the writer can write
	_sharedMemory->writer.wait();

	//create the original value
	_sharedMemory->msg = msg_tmp;

	// signal that the consumer can read msg
	_sharedMemory->reader.post();
}
// ###########################################

// Reads shared memory
template <typename T>
T SharedMemory<T>::readAsynchronously(){
	T tmp;

	// Only consumer may read from the shared memory
	if(_type!=SharedMemoryType::Consumer){
		assert(0);
	}

	_sharedMemory->writer.wait(); // waits until the producer signals that the consumer may read

	// here the process reads data from the shared memory!
	tmp=_sharedMemory->msg;

  	//writer can write
	_sharedMemory->writer.post();

	return tmp;
}

// ###########################################
template <typename T>
void SharedMemory<T>::writeAsynchronously(T msg_tmp){
	// Only producer can write to the shared memory
	if(_type!=SharedMemoryType::Producer){
		assert(0);
	}

	if(_sharedMemory==NULL){
		assert(0);
	}

	_sharedMemory->writer.wait();

	//create the original value
	_sharedMemory->msg = msg_tmp;

	// signal that the consumer can read msg
	_sharedMemory->writer.post();
}

// ###########################################

// ###########################################
template <typename T>
T SharedMemory<T>::readAsynchronouslyWriteSynchronously(){
	T tmp;

	// Only consumer may read from the shared memory
	if(_type!=SharedMemoryType::Consumer){
		assert(0);
	}


	_sharedMemory->reader.wait();

	tmp=_sharedMemory->msg; 	// here the process reads data from the shared memory!

	_sharedMemory->reader.post();
	_sharedMemory->writer.post();

	return tmp;
}

template <typename T>
void SharedMemory<T>::writeSynchronouslyReadAsynchronously(T msg_tmp){
// Only producer can write to the shared memory
	if(_type!=SharedMemoryType::Producer){
		assert(0);
	}

	if(_sharedMemory==NULL){
		assert(0);
	}

	_sharedMemory->writer.wait();
	_sharedMemory->reader.wait();

	_sharedMemory->msg = msg_tmp;

	_sharedMemory->reader.post();
}
// ###########################################

// ###########################################
template <typename T>
T SharedMemory<T>::readSynchronouslyWriteAsynchronously(){
	T tmp;

	// Only consumer may read from the shared memory
	if(_type!=SharedMemoryType::Consumer){
		assert(0);
	}


	_sharedMemory->reader.wait();
	_sharedMemory->writer.wait();

	tmp=_sharedMemory->msg; 	// here the process reads data from the shared memory!
	_sharedMemory->writer.post();

	return tmp;
}

template <typename T>
void SharedMemory<T>::writeAsynchronouslyReadSynchronously(T msg_tmp){
// Only producer can write to the shared memory
	if(_type!=SharedMemoryType::Producer){
		assert(0);
	}

	if(_sharedMemory==NULL){
		assert(0);
	}



	_sharedMemory->writer.wait();

	_sharedMemory->msg = msg_tmp;
	_sharedMemory->writer.post();

	_sharedMemory->reader.post();
}
// ###########################################

void testPrint(std::string str){
  std::cout<<"[DEBUG] - SHARED MEMORY - "<<str<<std::endl;
}

int function(int i){
    return 2*i;
}


template class SharedMemory<float>;
template class SharedMemory<int>;
template class SharedMemory<struct maxym>;
template class SharedMemory<struct lwr4_joints>;
template class SharedMemory<struct lwr4_kinematics_params>;
template class SharedMemory<struct lwr4_end_effector_position>;
