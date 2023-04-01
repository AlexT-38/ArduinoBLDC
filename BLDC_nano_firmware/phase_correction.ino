/* phase correction compensates for the winding RL filter
 *  the winding parameters are unkown and will need to be measured
 *  they can then be entered by the serial interface and stored in eeprom
 *  the table can then be recalculated at startup
 *  this may take some time due to the floating point math involved
 *  in which case we could have a selection of tables covering a range of
 *  cutoff frequencies (perhaps over the angular frequency range, ie 20-200)
 *  
 */
byte table_id = 12;
float table_f0;

unsigned int phase_correction_table[PHASE_TABLE_SIZE] = {0};

void init_phase_correction()
{
  //read the first byte of eeprom
  //if it's < no of tables, load that table into ram
  //otherwise generate the table using an f0 stored in the follwing 4 bytes of flash
  load_phase_table(table_id);
}

void clear_phase_table()
{
  table_id = NO_PHASE_TABLES;
  for(int idx=0; idx< PHASE_TABLE_SIZE; idx++)
  {
    PHASE(idx) = 0;
  }
}

void load_phase_table(byte table_no)
{
  if(table_no < NO_PHASE_TABLES)
  {
    table_id = table_no;
    unsigned int offset = PHASE_TABLE_OFFSET(table_no);
    for(int idx=0; idx< PHASE_TABLE_SIZE; idx++)
    {
      PHASE(idx) =  PHASE_TABLE(offset,idx);
    }
  }
  else if(table_no == NO_PHASE_TABLES)
  {
    clear_phase_table();
  }
}

void save_phase_table()
{
  
}

void print_phase_table()
{
  Serial.print("PhsTblF0: ");Serial.println(PHASE_TABLE_FREQ(table_id));
}
