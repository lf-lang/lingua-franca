#pragma once

/**
 * Source: http://www.cs.ucsb.edu/~franklin/20/assigns/prog2files/MySortedLinkedList.java
 */
template <typename T>
class SortedLinkedList {

private:
	/**
	 * stores a single item in the linked list
	 */
	template <typename U>
	class Node {
	public:
		U item;
		Node<U>* next;

		Node(U& i) {
			item = i;
			next = nullptr;
		}
	};

	// a reference to the first node in the list
	Node<T>* head;
	// a reference to the node to return when next() is called
	Node<T>* iterator;
public:
	/**
	 * constructor creates a linked list with no items in it
	 */
	SortedLinkedList() {
		head = nullptr;
		iterator = nullptr;
	}

	~SortedLinkedList() {
		std::vector<Node<T>*> elementsToDelete;

		Node<T>* n = head;
		while (n != nullptr) {
			elementsToDelete.push_back(n);
			n = n->next;
		}

		for(auto el: elementsToDelete) {
			delete el;
		}
	}

	/**
	 * isEmpty inputs: none return value: returns true if there are no items in linked list
	 */
	 bool isEmpty() {
		 return (head == nullptr);
	 }

	 /**
	  * add inputs: Comparable item return value: none adds an item into the list in sorted order
	  */
	 void add(T& item) {
		 // make the new node to insert into list
		 Node<T>* newNode = new Node(item);
		 // first see if the list is empty
		 if (head == nullptr) {
			 // std::cout << "add " << item << " to front";
			 head = newNode;
		 } else if (item < head->item) {
			 // there is something in the list
			 // now check to see if it belongs in front
			 // System.out.println("add "+item +"before"+head.item);
			 newNode->next = head;
			 head = newNode;
		 } else {
			 // otherwise, step down the list.  n will stop
			 // at the node after the new node, and trailer will
			 // stop at the node before the new node
			 Node<T>* after = head->next;
			 Node<T>* before = head;
			 while (after != nullptr) {
				 if (item < after->item) {
					 break;
				 }
				 before = after;
				 after = after->next;
			 }
			 // insert between before & after
			 newNode->next = before->next;
			 before->next = newNode;
			 // std::cout << "add " << item << "after" << before->item;
		 }
	 }

	 /* contains
	  * inputs: Comparable item
	  * return value: true if equal item is in list, false otherwise
	  */
	 bool contains(T& item) {
		 Node<T>* n = head;
		 // for each node in the linked list
		 while(n != nullptr) {
			 // if it is equal, return true
			 // note that I used compareTo here, not equals
			 // because I am only guaranteed that the
			 // compareTo method is implemented, not equals
			 if(item == n->item) {
				 return true;
			 }
			 n = n->next;
		 }
		 // if it is not found in list, return false
		 return false;
	 }

	 template <typename U>
	 friend std::ostream& operator<<(std::ostream&, SortedLinkedList&);

	 /**
	  * next inputs: none return value: one element from the linked list This method returns each element in the linked
	  * list in order. It is to be used in a loop to access every item in the list.
	  */
	 T* next() {
		 if (iterator != nullptr) {
			 Node<T>* n = iterator;
			 iterator = iterator->next;
			 return n->item;
		 } else {
			 return nullptr;
		 }
	 }

	 /**
	  * reset inputs: none return value: none resets the iterator so that the next call to next() will return the first
	  * element in the list
	  */
	 void reset() {
		 iterator = head;
	 }

	 /**
	  * size inputs: none return value: the number of elements in linked list
	  */
	 int size() {
		 int r = 0;
		 Node<T>* n = head;
		 // for each node in the linked list
		 while (n != nullptr) {
			 r++;
			 n = n->next;
		 }
		 return r;
	 }
};

/**
 * toString inputs: none return value: string representation of the linked list items Format must match assignment
 */
template <typename T>
std::ostream& operator<<(std::ostream& strm, SortedLinkedList<T>& a) {
	auto n = a.head;
	while(n != nullptr) {
		strm << n->item;
		n = n->next;
	}
	return strm;
}
